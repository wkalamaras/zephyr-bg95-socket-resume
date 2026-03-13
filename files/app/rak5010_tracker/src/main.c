/*
 * Copyright (c) 2023 Golioth, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(gps_tracking, LOG_LEVEL_DBG);

#include <zephyr/device.h>
#include <golioth/client.h>
#include <golioth/golioth_debug.h>
#include <golioth/lightdb_state.h>
#include <golioth/log.h>
#include <golioth/payload_utils.h>
#include <golioth/settings.h>
#include <golioth/stream.h>
#include <samples/common/sample_credentials.h>
#include <ctype.h>
#include <stdarg.h>
#include <stdlib.h>
#include <string.h>
#include <zcbor_encode.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/modem/quectel_bg9x.h>
#include <zephyr/drivers/watchdog.h>
#include <zephyr/drivers/hwinfo.h>
#include <zephyr/settings/settings.h>
#include <zephyr/kernel.h>
#if defined(CONFIG_MODEM_CELLULAR)
#include <zephyr/modem/chat.h>
#include <zephyr/modem/pipe.h>
#include <zephyr/modem/pipelink.h>
#endif
#include <zephyr/net/net_if.h>
#include <zephyr/net/net_mgmt.h>
#include <zephyr/net/socket.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/reboot.h>
#include <zephyr/data/json.h>
#include <zephyr/shell/shell.h>
#include <zephyr/shell/shell_dummy.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/util.h>
#include <samples/common/net_connect.h>

#define APP_GOLIOTH_LOG_MODULE "GPS Tracking"
#define APP_WDT_TIMEOUT_SECONDS CONFIG_APP_WDT_TIMEOUT_SECONDS
#define APP_STATE_TIMEOUT_SECONDS 10
#define APP_LOG_TIMEOUT_SECONDS 30
#define TRACKER_STREAM_TIMEOUT_SECONDS 60
#define APP_DEBUG_CONNECT_TIMEOUT_SECONDS 90
#define APP_DEBUG_WORKQ_STACK_SIZE 8192
#define APP_DEBUG_WORKQ_PRIORITY 5
#define APP_UART1_BRIDGE_STACK_SIZE 2048
#define APP_UART1_BRIDGE_PRIORITY 7
#define APP_UART1_BRIDGE_LINE_MAX 128
#define POST_CONNECT_SETTLE_MS 3000
#define SETTINGS_QUIET_WINDOW_MS 1500
#define SETTINGS_QUIET_MAX_WAIT_MS 6000
#define APP_TRACKER_STATE_ROOT "tracker"
#define APP_CMD_PENDING_ROOT "cmd/pending"
#define APP_CMD_RESULT_ROOT "cmd/results"
#define APP_STATE_PATH_MAX 64
#define APP_MAILBOX_MAX_COMMANDS 8
#define APP_MAILBOX_ID_MAX 24
#define APP_CMD_JSON_MAX 384
#define APP_CMD_RESULT_MAX 256
#define APP_MAILBOX_PENDING_MAX 768
#define LIS2DH_MOTION_FILTER_CFG 0x81
#define MODEM_DIAG_READY_TIMEOUT_SECONDS 90
#define APP_DEBUG_BUILD_TAG "dbgtrk-inline-v3"

static const struct pwm_dt_spec app_led = PWM_DT_SPEC_GET(DT_ALIAS(pwm_led0));
static const struct device *const app_uart1 = DEVICE_DT_GET(DT_NODELABEL(uart1));
static struct k_thread app_uart1_bridge_thread;
K_THREAD_STACK_DEFINE(app_uart1_bridge_stack, APP_UART1_BRIDGE_STACK_SIZE);
static bool app_uart1_bridge_started;

/* GPS antenna power MOSFET — RAK5010 P1.07, HIGH = antenna ON */
static const struct device *const gps_ant_gpio = DEVICE_DT_GET(DT_NODELABEL(gpio1));
#define GPS_ANT_PIN 7

static const struct device *const sensor_accel = DEVICE_DT_GET(DT_ALIAS(accel0));
static const struct device *const sensor_press = DEVICE_DT_GET_ANY(st_lps22hb_press);
static const struct device *const sensor_light = DEVICE_DT_GET_ANY(ti_opt3001);
static const struct device *const sensor_shtc3 = DEVICE_DT_GET(DT_NODELABEL(shtc3));

#if defined(CONFIG_MODEM_CELLULAR)
#define MODEM_DIAG_TIMEOUT_SECONDS 8
#define MODEM_PSM_SET_CMD \
	"AT+CPSMS=1,,,\"" CONFIG_MODEM_QUECTEL_BG9X_PSM_TAU "\",\"" \
	CONFIG_MODEM_QUECTEL_BG9X_PSM_ACTIVE_TIME "\""

#define MODEM_NODE DT_ALIAS(modem)
MODEM_PIPELINK_DT_DECLARE(MODEM_NODE, user_pipe_1);

enum modem_diag_state_bits {
	MODEM_DIAG_ATTACHED_BIT = 0,
	MODEM_DIAG_BUSY_BIT,
};

struct modem_diag_ctx {
	struct modem_pipelink *pipelink;
	struct modem_chat chat;
	struct k_work open_pipe_work;
	struct k_work attach_chat_work;
	struct k_work release_chat_work;
	struct k_sem attached_sem;
	struct k_sem done_sem;
	atomic_t state;
	enum modem_chat_script_result last_result;
	uint8_t chat_receive_buf[128];
	uint8_t *chat_argv_buf[2];
	uint8_t request_buf[96];
	uint8_t match_buf[32];
	struct modem_chat_script_chat script_chat[1];
	struct modem_chat_match script_matches[2];
};

static struct modem_diag_ctx modem_diag = {
	.pipelink = MODEM_PIPELINK_DT_GET(MODEM_NODE, user_pipe_1),
};
#endif

static struct golioth_client *client;
static const struct golioth_client_config *app_client_config;
static struct golioth_settings *app_golioth_settings_ctx;
static K_SEM_DEFINE(connected, 0, 1);
static struct k_mutex app_settings_cb_lock;
static const struct device *const app_wdt = DEVICE_DT_GET(DT_ALIAS(watchdog0));
static int app_wdt_channel_id = -1;
static uint32_t app_boot_count;
static uint32_t app_total_cycles;
static uint32_t app_cycle_tx_bytes;
static uint32_t app_cycle_aux_tx_bytes;
static uint32_t app_cycle_tracker_tx_bytes;
static uint32_t app_bytes_today;
static uint32_t app_day_boundary;
static bool app_day_boundary_valid;
static uint32_t app_last_tracker_payload_len;
static uint32_t app_last_tracker_path_len;
static uint32_t app_last_tracker_tx_estimate;
static uint32_t app_last_cycle_log_len;
static char app_last_tracker_path[32];
static struct bg9x_cell_info app_last_valid_cell_info;
static bool app_last_valid_cell_info_present;
static struct bg9x_network_time app_last_valid_network_time;
static bool app_last_valid_network_time_present;
#if defined(CONFIG_APP_LOG_RAW_TRACKER_PAYLOAD)
static char app_last_tracker_payload[768];
#endif
static uint32_t app_psm_disabled = IS_ENABLED(CONFIG_APP_PSM_DISABLED_DEFAULT) ? 1U : 0U;
static uint32_t app_gps_enabled =
	IS_ENABLED(CONFIG_APP_GPS_ENABLED_DEFAULT) ? 1U : 0U; /* 0=skip GPS, 1=attempt GPS */
static uint32_t app_tracking_interval_seconds = CONFIG_APP_SLEEP_SECONDS;
static uint32_t app_parked_interval_seconds = CONFIG_APP_PARKED_HEARTBEAT_SECONDS;
static uint32_t app_motion_threshold_mg = CONFIG_APP_MOTION_THRESHOLD_MG;
static uint32_t app_motion_duration_samples = CONFIG_APP_MOTION_DURATION_SAMPLES;
static uint32_t app_tracking_speed_threshold_kmh = CONFIG_APP_TRACKING_SPEED_THRESHOLD_KMH;
static uint32_t app_tracking_stop_count = CONFIG_APP_TRACKING_STOP_COUNT;
static uint32_t app_gps_timeout_seconds = 180U;
static uint32_t app_cellular_timeout_seconds = CONFIG_APP_CONNECT_TIMEOUT_SECONDS;
static uint32_t app_wake_anchored_intervals = 1U;
static uint32_t app_cbor_enabled =
	IS_ENABLED(CONFIG_APP_CBOR_ENABLED_DEFAULT) ? 1U : 0U;
static uint32_t app_minified_payload =
	IS_ENABLED(CONFIG_APP_MINIFIED_PAYLOAD_DEFAULT) ? 1U : 0U;
static uint32_t app_xtra_enabled =
	IS_ENABLED(CONFIG_APP_XTRA_ENABLED_DEFAULT) ? 1U : 0U;
static uint32_t app_xtra_retention_days = CONFIG_APP_XTRA_RETENTION_DEFAULT;
static uint32_t app_xtra_mode = CONFIG_APP_XTRA_MODE_DEFAULT;
static char app_pending_status_marker;
static bool app_runtime_cfg_dirty = true;
static bool app_legacy_cfg_state_pruned;
static bool app_legacy_command_state_pruned;
static uint32_t app_tracking_motion_events_last_interval;
static bool app_reboot_requested;
static bool app_force_cycle_requested;
static bool app_force_cycle_no_psm_requested;
static bool app_cycle_no_psm_active;
static bool app_manual_cycle_idle_announced;
static bool app_cloud_cycle_log_enabled = true;
static bool app_settings_sync_enabled = false;
static bool app_mailbox_poll_enabled = false;
static bool app_last_data_usage_valid;
static struct bg9x_data_usage app_last_data_usage;
static bool app_cycle_data_usage_valid;
static struct bg9x_data_usage app_cycle_data_usage_start;
static struct bg9x_data_usage app_cycle_data_usage_last;
static const char *app_cycle_data_usage_last_phase;
static uint32_t app_debug_probe_counter;
static char app_debug_probe_path[32] = "diag/probe";
K_THREAD_STACK_DEFINE(app_debug_workq_stack, APP_DEBUG_WORKQ_STACK_SIZE);
static struct k_work_q app_debug_workq;
static bool app_debug_workq_started;
static struct k_work app_debug_sendtracker_work;
static atomic_t app_debug_sendtracker_busy;
static struct k_work_delayable app_debug_sendtracker_autorun_work;
static struct k_work app_debug_resumetracker_work;
static atomic_t app_debug_resumetracker_busy;
static uint32_t app_debug_resumetracker_idle_seconds;
static uint32_t app_debug_resumetracker_count = 1U;
static bool app_debug_resumetracker_autotest_scheduled;
static char app_mailbox_pending_json[APP_MAILBOX_PENDING_MAX];
static size_t app_mailbox_pending_json_len;

struct app_mailbox_command_id {
	char value[APP_MAILBOX_ID_MAX];
};

struct app_setting_u32_cache {
	bool valid;
	uint32_t value;
};

struct app_setting_bool_cache {
	bool valid;
	bool value;
};

static struct app_setting_u32_cache app_cloud_tracking_interval_cache;
static struct app_setting_u32_cache app_cloud_parked_interval_cache;
static struct app_setting_u32_cache app_cloud_motion_threshold_cache;
static struct app_setting_u32_cache app_cloud_motion_duration_cache;
static struct app_setting_u32_cache app_cloud_tracking_speed_cache;
static struct app_setting_u32_cache app_cloud_tracking_stop_cache;
static struct app_setting_u32_cache app_cloud_gps_timeout_cache;
static struct app_setting_u32_cache app_cloud_cellular_timeout_cache;
static struct app_setting_u32_cache app_cloud_xtra_retention_cache;
static struct app_setting_u32_cache app_cloud_xtra_mode_cache;
static struct app_setting_bool_cache app_cloud_wake_anchored_cache;
static struct app_setting_bool_cache app_cloud_gps_enabled_cache;
static struct app_setting_bool_cache app_cloud_psm_enabled_cache;
static struct app_setting_bool_cache app_cloud_cbor_enabled_cache;
static struct app_setting_bool_cache app_cloud_minified_payload_cache;
static struct app_setting_bool_cache app_cloud_xtra_enabled_cache;
static int64_t app_last_settings_activity_ms;
static void app_wdt_feed(void);
static void on_client_event(struct golioth_client *client,
			    enum golioth_client_event event,
			    void *arg);
static void app_debug_sendtracker_autorun_work_handler(struct k_work *work);
static void app_debug_sendtracker_work_handler(struct k_work *work);
static void app_debug_resumetracker_work_handler(struct k_work *work);
static int app_debug_connect_client(uint32_t timeout_seconds);
static int app_debug_send_tracker_from_cache(void);
static void app_debug_log_modemsnap(const char *tag, bool probe_at);
static int app_debug_prepare_resume_socket(const char *tag);
static void app_uart1_write_text(const char *text);
static void app_uart1_write_line(const char *line);
static void app_uart1_bridge_thread_fn(void *p1, void *p2, void *p3);
static void app_uart1_bridge_start(void);
static int app_uart1_execute_line(const char *line);
static void app_debug_seed_tracker_cache(int counter);
static int cmd_at(const struct shell *sh, size_t argc, char **argv);
static int cmd_atq(const struct shell *sh, size_t argc, char **argv);
static int cmd_qgdcnt(const struct shell *sh, size_t argc, char **argv);
static int cmd_modemsnap(const struct shell *sh, size_t argc, char **argv);
static int cmd_uart1probe(const struct shell *sh, size_t argc, char **argv);
static int cmd_connectonly(const struct shell *sh, size_t argc, char **argv);
static int cmd_disconnect(const struct shell *sh, size_t argc, char **argv);
static int cmd_resume_socket(const struct shell *sh, size_t argc, char **argv);
static int cmd_sendprobe(const struct shell *sh, size_t argc, char **argv);
static int cmd_sendtracker(const struct shell *sh, size_t argc, char **argv);
static int cmd_resumeprobe(const struct shell *sh, size_t argc, char **argv);
static int cmd_resumeprobe_nopsm(const struct shell *sh, size_t argc, char **argv);
static int cmd_resumetracker(const struct shell *sh, size_t argc, char **argv);
static int cmd_settingssync(const struct shell *sh, size_t argc, char **argv);
static int cmd_mailboxpoll(const struct shell *sh, size_t argc, char **argv);
static int cmd_cyclelog(const struct shell *sh, size_t argc, char **argv);

static void app_note_settings_activity(void)
{
	app_last_settings_activity_ms = k_uptime_get();
}

static bool app_golioth_session_idle(bool require_settings_idle)
{
	uint32_t queued;
	uint32_t inflight;
	bool settings_busy = false;
	int64_t last_ms;

	if (client == NULL || !golioth_client_is_connected(client)) {
		return false;
	}

	queued = golioth_client_num_items_in_request_queue(client);
	inflight = golioth_client_num_in_flight_requests(client);
	if (require_settings_idle && app_golioth_settings_ctx != NULL) {
		settings_busy =
			golioth_settings_request_in_flight(app_golioth_settings_ctx);
	}

	if (queued != 0U || inflight != 0U || settings_busy) {
		return false;
	}

	if (!require_settings_idle) {
		return true;
	}

	last_ms = app_last_settings_activity_ms;
	return last_ms == 0 ||
	       (k_uptime_get() - last_ms) >= SETTINGS_QUIET_WINDOW_MS;
}

static void app_uart1_write_text(const char *text)
{
	if (!device_is_ready(app_uart1) || text == NULL) {
		return;
	}

	while (*text != '\0') {
		uart_poll_out(app_uart1, (unsigned char)*text++);
	}
}

static void app_uart1_write_line(const char *line)
{
	if (line == NULL) {
		return;
	}

	app_uart1_write_text(line);

	uart_poll_out(app_uart1, '\r');
	uart_poll_out(app_uart1, '\n');
}

struct app_uart1_cmd_entry {
	const char *name;
	int (*handler)(const struct shell *sh, size_t argc, char **argv);
};

static int app_uart1_execute_line(const char *line)
{
	const struct shell *dummy = shell_backend_dummy_get_ptr();
	static const struct app_uart1_cmd_entry cmd_table[] = {
		{ "at", cmd_at },
		{ "atq", cmd_atq },
		{ "qgdcnt", cmd_qgdcnt },
		{ "modemsnap", cmd_modemsnap },
		{ "uart1probe", cmd_uart1probe },
		{ "connectonly", cmd_connectonly },
		{ "disconnect", cmd_disconnect },
		{ "resume_socket", cmd_resume_socket },
		{ "sendprobe", cmd_sendprobe },
		{ "sendtracker", cmd_sendtracker },
		{ "resumeprobe", cmd_resumeprobe },
		{ "resumeprobe_nopsm", cmd_resumeprobe_nopsm },
		{ "resumetracker", cmd_resumetracker },
		{ "settingssync", cmd_settingssync },
		{ "mailboxpoll", cmd_mailboxpoll },
		{ "cyclelog", cmd_cyclelog },
	};
	char buf[APP_UART1_BRIDGE_LINE_MAX];
	char *argv[12];
	char *ctx = NULL;
	char *tok;
	size_t argc = 0U;
	size_t i;

	if (line == NULL || line[0] == '\0') {
		return 0;
	}

	snprintk(buf, sizeof(buf), "%s", line);
	for (tok = strtok_r(buf, " \t", &ctx);
	     tok != NULL && argc < ARRAY_SIZE(argv);
	     tok = strtok_r(NULL, " \t", &ctx)) {
		argv[argc++] = tok;
	}

	if (argc == 0U) {
		return 0;
	}

	for (i = 0; i < ARRAY_SIZE(cmd_table); ++i) {
		size_t out_len = 0U;
		const char *out;
		int ret;

		if (strcmp(argv[0], cmd_table[i].name) != 0) {
			continue;
		}

		shell_backend_dummy_clear_output(dummy);
		ret = cmd_table[i].handler(dummy, argc, argv);
		out = shell_backend_dummy_get_output(dummy, &out_len);
		if (out != NULL && out_len > 0U) {
			char out_buf[CONFIG_SHELL_BACKEND_DUMMY_BUF_SIZE + 1];
			size_t copy_len = MIN(out_len, sizeof(out_buf) - 1U);

			memcpy(out_buf, out, copy_len);
			out_buf[copy_len] = '\0';
			app_uart1_write_text(out_buf);
			if (copy_len > 0U && out_buf[copy_len - 1U] != '\n') {
				app_uart1_write_text("\r\n");
			}
		}

		return ret;
	}

	app_uart1_write_line("ERR unknown");
	return -ENOENT;
}

static void app_uart1_bridge_thread_fn(void *p1, void *p2, void *p3)
{
	char line[APP_UART1_BRIDGE_LINE_MAX];
	size_t len = 0U;

	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	if (!device_is_ready(app_uart1)) {
		return;
	}

	app_uart1_write_line("UART1 bridge ready");

	while (true) {
		unsigned char ch;
		int ret = uart_poll_in(app_uart1, &ch);

		if (ret < 0) {
			k_sleep(K_MSEC(10));
			continue;
		}

		if (ch == '\r' || ch == '\n') {
			int cmd_ret;

			if (len == 0U) {
				continue;
			}

			line[len] = '\0';
			app_uart1_write_text("> ");
			app_uart1_write_line(line);
			cmd_ret = app_uart1_execute_line(line);
			if (cmd_ret != 0) {
				char err_line[48];

				snprintk(err_line, sizeof(err_line), "ERR %d", cmd_ret);
				app_uart1_write_line(err_line);
			}
			len = 0U;
			continue;
		}

		if (ch == '\b' || ch == 0x7f) {
			if (len > 0U) {
				len--;
			}
			continue;
		}

		if (ch < 0x20 || ch > 0x7e) {
			continue;
		}

		if (len < (sizeof(line) - 1U)) {
			line[len++] = (char)ch;
		}
	}
}

static void app_uart1_bridge_start(void)
{
	if (app_uart1_bridge_started || !device_is_ready(app_uart1)) {
		return;
	}

	k_thread_create(&app_uart1_bridge_thread, app_uart1_bridge_stack,
			K_THREAD_STACK_SIZEOF(app_uart1_bridge_stack),
			app_uart1_bridge_thread_fn,
			NULL, NULL, NULL,
			APP_UART1_BRIDGE_PRIORITY, 0, K_NO_WAIT);
	k_thread_name_set(&app_uart1_bridge_thread, "uart1_bridge");
	app_uart1_bridge_started = true;
}

static void app_debug_workq_init(void)
{
	if (app_debug_workq_started) {
		return;
	}

	k_work_queue_init(&app_debug_workq);
	k_work_queue_start(&app_debug_workq, app_debug_workq_stack,
			   K_THREAD_STACK_SIZEOF(app_debug_workq_stack),
			   APP_DEBUG_WORKQ_PRIORITY, NULL);
	k_thread_name_set(&app_debug_workq.thread, "app_dbg_wq");
	app_debug_workq_started = true;
}

static void app_wait_for_golioth_idle(const char *reason, bool require_settings_idle)
{
	int64_t start_ms = k_uptime_get();
	int64_t last_ms;
	uint32_t queued = 0U;
	uint32_t inflight = 0U;
	bool settings_busy = false;

	/* Give the newly established session a brief chance to deliver any
	 * pending settings callbacks before the first tracker publish.
	 */
	k_sleep(K_MSEC(POST_CONNECT_SETTLE_MS));

	while (true) {
		if (app_golioth_session_idle(require_settings_idle)) {
			return;
		}

		last_ms = app_last_settings_activity_ms;
		if (client != NULL && golioth_client_is_connected(client)) {
			queued = golioth_client_num_items_in_request_queue(client);
			inflight = golioth_client_num_in_flight_requests(client);
		}
		if (require_settings_idle && app_golioth_settings_ctx != NULL) {
			settings_busy =
				golioth_settings_request_in_flight(app_golioth_settings_ctx);
		}

		if ((k_uptime_get() - start_ms) >= SETTINGS_QUIET_MAX_WAIT_MS) {
			LOG_WRN("golioth idle wait timed out after %dms reason=%s queued=%u inflight=%u settings_busy=%u last_settings_ms=%lld",
				SETTINGS_QUIET_MAX_WAIT_MS,
				reason != NULL ? reason : "n/a",
				(unsigned int)queued,
				(unsigned int)inflight,
				settings_busy ? 1U : 0U,
				(long long)last_ms);
			return;
		}

		app_wdt_feed();
		k_sleep(K_MSEC(100));
	}
}

static void app_refresh_connected_network_snapshot(int counter,
					  struct bg9x_cell_info *cell,
					  struct bg9x_network_time *ntime)
{
	struct bg9x_cell_info refreshed_cell = {0};
	struct bg9x_network_time refreshed_time = {0};

	if (bg9x_cell_info_get(&refreshed_cell) == 0) {
		if (refreshed_cell.valid &&
		    refreshed_cell.rat[0] != '\0' &&
		    refreshed_cell.mcc != 0U &&
		    refreshed_cell.mnc != 0U &&
		    refreshed_cell.cell_id != 0U &&
		    refreshed_cell.earfcn != 0U &&
		    refreshed_cell.band != 0U &&
		    !(refreshed_cell.rsrp == 0 &&
		      refreshed_cell.rsrq == 0 &&
		      refreshed_cell.sinr == 0)) {
			*cell = refreshed_cell;
			app_last_valid_cell_info = refreshed_cell;
			app_last_valid_cell_info_present = true;
			LOG_INF("#%d cell(refresh): RAT=%s MCC=%u MNC=%u cellID=%x RSRP=%d SINR=%d",
				counter, cell->rat, cell->mcc, cell->mnc,
				cell->cell_id, cell->rsrp, cell->sinr);
		} else if (app_last_valid_cell_info_present) {
			*cell = app_last_valid_cell_info;
			LOG_DBG("#%d cell(refresh): ignoring incomplete modem snapshot, keeping cellID=%x",
				counter, cell->cell_id);
		} else {
			memset(cell, 0, sizeof(*cell));
			LOG_DBG("#%d cell(refresh): ignoring incomplete modem snapshot with no cache",
				counter);
		}
	}

	if (bg9x_network_time_get(&refreshed_time) == 0 && refreshed_time.valid) {
		*ntime = refreshed_time;
		app_last_valid_network_time = refreshed_time;
		app_last_valid_network_time_present = true;
		LOG_INF("#%d net_time(refresh): %04u-%02u-%02uT%02u:%02u:%02u tz=%d",
			counter, ntime->year, ntime->month,
			ntime->day, ntime->hour, ntime->min,
			ntime->sec, ntime->tz_quarter_hours / 4);
	} else if (app_last_valid_network_time_present) {
		*ntime = app_last_valid_network_time;
	}
}

static void app_seed_cloud_setting_caches(void)
{
	app_cloud_tracking_interval_cache = (struct app_setting_u32_cache){
		.valid = true,
		.value = app_tracking_interval_seconds,
	};
	app_cloud_parked_interval_cache = (struct app_setting_u32_cache){
		.valid = true,
		.value = app_parked_interval_seconds,
	};
	app_cloud_motion_threshold_cache = (struct app_setting_u32_cache){
		.valid = true,
		.value = app_motion_threshold_mg,
	};
	app_cloud_motion_duration_cache = (struct app_setting_u32_cache){
		.valid = true,
		.value = app_motion_duration_samples,
	};
	app_cloud_tracking_speed_cache = (struct app_setting_u32_cache){
		.valid = true,
		.value = app_tracking_speed_threshold_kmh,
	};
	app_cloud_tracking_stop_cache = (struct app_setting_u32_cache){
		.valid = true,
		.value = app_tracking_stop_count,
	};
	app_cloud_gps_timeout_cache = (struct app_setting_u32_cache){
		.valid = true,
		.value = app_gps_timeout_seconds,
	};
	app_cloud_cellular_timeout_cache = (struct app_setting_u32_cache){
		.valid = true,
		.value = app_cellular_timeout_seconds,
	};
	app_cloud_xtra_retention_cache = (struct app_setting_u32_cache){
		.valid = true,
		.value = app_xtra_retention_days,
	};
	app_cloud_xtra_mode_cache = (struct app_setting_u32_cache){
		.valid = true,
		.value = app_xtra_mode,
	};
	app_cloud_wake_anchored_cache = (struct app_setting_bool_cache){
		.valid = true,
		.value = app_wake_anchored_intervals != 0U,
	};
	app_cloud_gps_enabled_cache = (struct app_setting_bool_cache){
		.valid = true,
		.value = app_gps_enabled != 0U,
	};
	app_cloud_psm_enabled_cache = (struct app_setting_bool_cache){
		.valid = true,
		.value = app_psm_disabled == 0U,
	};
	app_cloud_cbor_enabled_cache = (struct app_setting_bool_cache){
		.valid = true,
		.value = app_cbor_enabled != 0U,
	};
	app_cloud_minified_payload_cache = (struct app_setting_bool_cache){
		.valid = true,
		.value = app_minified_payload != 0U,
	};
	app_cloud_xtra_enabled_cache = (struct app_setting_bool_cache){
		.valid = true,
		.value = app_xtra_enabled != 0U,
	};
}

static enum golioth_status delete_lightdb_state(const char *path);

static bool app_status_marker_cloud_enabled(void)
{
	/* Keep status markers local-only on the production tracker path. */
	return !(app_cbor_enabled != 0U && app_minified_payload != 0U);
}

enum tracker_mode {
	TRACKER_PARKED = 0,
	TRACKER_TRACKING = 1,
};

static enum tracker_mode app_tracker_mode =
	IS_ENABLED(CONFIG_APP_TRACKER_START_PARKED) ? TRACKER_PARKED : TRACKER_TRACKING;
static int app_consecutive_slow_fixes;
static volatile bool app_motion_detected;
static volatile bool app_tracking_motion_seen;
static volatile uint32_t app_tracking_motion_events;
static bool app_motion_trigger_ready;
static struct sensor_trigger app_motion_trigger = {
	.type = SENSOR_TRIG_DELTA,
	.chan = SENSOR_CHAN_ACCEL_XYZ,
};

/* Last GPS fix — stored in settings, restored on boot */
static int32_t app_gps_lat_e6;    /* latitude * 1e6 */
static int32_t app_gps_lon_e6;    /* longitude * 1e6 */
static int32_t app_gps_alt_cm;    /* altitude in centimeters */
static uint32_t app_gps_speed_x10; /* speed_kmh * 10 */
static uint32_t app_gps_course_x10; /* course * 10 */
static uint32_t app_gps_sats;
static uint32_t app_gps_fixtime_ms;
static uint32_t app_gps_uptime_ms;

/* Last sensor readings — filled by read_sensors(), used by log payload */
static int32_t last_accel_x, last_accel_y, last_accel_z; /* mg */
static int32_t last_temp_c10;   /* temp * 10 (e.g. 221 = 22.1C) */
static int32_t last_humid_c10;  /* humidity * 10 */
static int32_t last_batt_pct = -1;
static int32_t last_batt_mv = -1;
static int32_t last_pressure_hpa10; /* pressure in hPa * 10 */
static int32_t last_light_lux;
static bool pressure_sensor_available;
static bool light_sensor_available;
static bool pressure_sensor_absent_logged;
static bool light_sensor_absent_logged;
static bool optional_sensor_state_pruned;

enum app_led_pattern {
	APP_LED_OFF = 0,
	APP_LED_BOOT,
	APP_LED_GPS_POWER_ON,
	APP_LED_GPS_ACQUIRING,
	APP_LED_GPS_FIX,
	APP_LED_MOTION_WAKE,
	APP_LED_TRACKING_MOTION,
	APP_LED_CONNECTED,
	APP_LED_PAYLOAD_SENT,
	APP_LED_BREATHE,
	APP_LED_BREATHE50,
	APP_LED_HEARTBEAT,
	APP_LED_CHARGING,
	APP_LED_ERROR,
};

#define TRACKER_FIELD_ALIAS_MAP(X) \
	X(CYCLE, "cy", "cycle") \
	X(BOOT_COUNT, "bt", "boot_count") \
	X(TOTAL_CYCLES, "tc", "total_cycles") \
	X(UPTIME_MS, "up", "uptime_ms") \
	X(HEARTBEAT, "hb", "heartbeat") \
	X(MODE, "md", "mode") \
	X(NEXT_WAKE_S, "nx", "next_wake_s") \
	X(MOTION_SEEN, "ms", "motion_seen") \
	X(TEMP_C_X10, "tm", "temp_c_x10") \
	X(HUMIDITY_X10, "hu", "humidity_x10") \
	X(ACCEL_X, "ax", "accel_x") \
	X(ACCEL_Y, "ay", "accel_y") \
	X(ACCEL_Z, "az", "accel_z") \
	X(BAT_PCT, "bp", "bat_pct") \
	X(BAT_MV, "bx", "bat_mv") \
	X(EVENT_TIME, "ts", "event_time") \
	X(GPS_TIME, "gt", "gps_time") \
	X(NET_TIME, "nt", "net_time") \
	X(TZ, "tz", "tz") \
	X(LAT_E6, "la", "lat_e6") \
	X(LON_E6, "lo", "lon_e6") \
	X(ALT_DM, "al", "alt_dm") \
	X(SPEED_X10, "sp", "speed_x10") \
	X(HEADING_X10, "hd", "heading_x10") \
	X(FIX_TYPE, "fx", "fix_type") \
	X(FIX_TIME_MS, "ft", "fix_time_ms") \
	X(SATS_USED, "su", "sats_used") \
	X(SATS_VIEW, "sv", "sats_view") \
	X(HDOP_X10, "hp", "hdop_x10") \
	X(PDOP_X10, "pd", "pdop_x10") \
	X(VDOP_X10, "vd", "vdop_x10") \
	X(MAG_VAR_X10, "mv", "mag_var_x10") \
	X(RSRP, "rs", "rsrp") \
	X(RSRQ, "rq", "rsrq") \
	X(SINR, "sn", "sinr") \
	X(CELL_ID, "ci", "cell_id") \
	X(PCI, "pc", "pci") \
	X(EARFCN, "ea", "earfcn") \
	X(MCC, "mc", "mcc") \
	X(MNC, "mn", "mnc") \
	X(BAND, "bd", "band") \
	X(BG95_NATIVE_WAKE_COUNT, "nw", "bg95_native_wake_count") \
	X(BG95_EMERGENCY_REINIT_COUNT, "ew", "bg95_emergency_reinit_count")

enum tracker_field_id {
#define TRACKER_FIELD_ENUM(id, short_key, long_key) TRACKER_FIELD_##id,
	TRACKER_FIELD_ALIAS_MAP(TRACKER_FIELD_ENUM)
#undef TRACKER_FIELD_ENUM
};

struct app_led_step {
	uint8_t brightness_pct;
	uint16_t duration_ms;
};

static const struct app_led_step app_led_pattern_boot[] = {
	{ 100, 1000 }, { 0, 0 },
};
static const struct app_led_step app_led_pattern_gps_power_on[] = {
	{ 1, 120 }, { 0, 0 },
};
static const struct app_led_step app_led_pattern_gps_acquiring[] = {
	{ 100, 100 }, { 0, 900 }, { 0, UINT16_MAX },
};
static const struct app_led_step app_led_pattern_gps_fix[] = {
	{ 100, 5000 }, { 0, 0 },
};
static const struct app_led_step app_led_pattern_motion_wake[] = {
	{ 100, 120 }, { 0, 120 },
	{ 100, 120 }, { 0, 120 },
	{ 100, 120 }, { 0, 240 },
	{ 100, 3000 }, { 0, 0 },
};
static const struct app_led_step app_led_pattern_tracking_motion[] = {
	{ 1, 120 }, { 0, 120 },
	{ 1, 120 }, { 0, 0 },
};
static const struct app_led_step app_led_pattern_connected[] = {
	{ 0, 500 },
	{ 1, 120 }, { 0, 120 },
	{ 1, 120 }, { 0, 120 },
	{ 1, 120 }, { 0, 0 },
};
static const struct app_led_step app_led_pattern_payload_sent[] = {
	{ 0, 120 },
	{ 18, 40 }, { 48, 50 }, { 82, 70 }, { 24, 70 }, { 0, 0 },
};
static const struct app_led_step app_led_pattern_heartbeat[] = {
	{ 0, 140 },
	{ 18, 40 }, { 48, 50 }, { 82, 70 }, { 24, 70 }, { 0, 80 },
	{ 24, 40 }, { 65, 50 }, { 100, 90 }, { 28, 90 }, { 0, 720 },
	{ 0, UINT16_MAX },
};
static const struct app_led_step app_led_pattern_error[] = {
	{ 100, 200 }, { 0, 200 },
	{ 100, 200 }, { 0, 200 },
	{ 100, 200 }, { 0, 400 },
	{ 100, 600 }, { 0, 200 },
	{ 100, 600 }, { 0, 200 },
	{ 100, 600 }, { 0, 400 },
	{ 100, 200 }, { 0, 200 },
	{ 100, 200 }, { 0, 200 },
	{ 100, 200 }, { 0, 0 },
};

static struct k_work_delayable app_led_work;
static const struct app_led_step *app_led_steps;
static size_t app_led_step_index;
static bool app_led_ready;
static bool app_led_manual_override;
static uint8_t app_led_breathe_brightness;
static uint8_t app_led_breathe_max;
static bool app_led_breathe_rising;
static enum app_led_pattern app_led_current_pattern = APP_LED_OFF;
static bool app_tz_qh_deleted;
static bool app_boot_from_software_reset;

static void app_wdt_feed(void);
static int app_motion_trigger_init(void);

static uint32_t app_next_wake_seconds(bool heartbeat_only)
{
	ARG_UNUSED(heartbeat_only);
	return (app_tracker_mode == TRACKER_TRACKING) ? app_tracking_interval_seconds :
		app_parked_interval_seconds;
}

static void app_log_reset_cause(void)
{
	uint32_t cause = 0U;
	int ret;

	ret = hwinfo_get_reset_cause(&cause);
	if (ret == -ENOSYS) {
		LOG_INF("reset cause unsupported");
		return;
	}

	if (ret < 0) {
		LOG_WRN("reset cause read failed: %d", ret);
		return;
	}

	app_boot_from_software_reset = (cause & RESET_SOFTWARE) != 0U;

	LOG_INF("reset cause raw=0x%08x%s%s%s%s%s%s%s",
		cause,
		(cause & RESET_WATCHDOG) ? " WATCHDOG" : "",
		(cause & RESET_SOFTWARE) ? " SOFTWARE" : "",
		(cause & RESET_POR) ? " POR" : "",
		(cause & RESET_PIN) ? " PIN" : "",
		(cause & RESET_LOW_POWER_WAKE) ? " LOW_POWER_WAKE" : "",
		(cause & RESET_CPU_LOCKUP) ? " CPU_LOCKUP" : "",
		(cause & RESET_BROWNOUT) ? " BROWNOUT" : "");

	(void)hwinfo_clear_reset_cause();
}

static void app_led_set_brightness(uint8_t brightness_pct)
{
	uint32_t pulse;

	if (!app_led_ready) {
		return;
	}

	if (brightness_pct == 0U) {
		pulse = app_led.period;
	} else if (brightness_pct >= 100U) {
		pulse = 0U;
	} else {
		/* With PWM_POLARITY_INVERTED on this board, lower pulse means brighter LED. */
		pulse = (uint32_t)(((uint64_t)app_led.period * (100U - brightness_pct)) / 100U);
	}

	(void)pwm_set_dt(&app_led, app_led.period, pulse);
}

static const struct app_led_step *app_led_pattern_steps(enum app_led_pattern pattern)
{
	switch (pattern) {
	case APP_LED_BOOT:
		return app_led_pattern_boot;
	case APP_LED_GPS_POWER_ON:
		return app_led_pattern_gps_power_on;
	case APP_LED_GPS_ACQUIRING:
		return app_led_pattern_gps_acquiring;
	case APP_LED_GPS_FIX:
		return app_led_pattern_gps_fix;
	case APP_LED_MOTION_WAKE:
		return app_led_pattern_motion_wake;
	case APP_LED_TRACKING_MOTION:
		return app_led_pattern_tracking_motion;
	case APP_LED_CONNECTED:
		return app_led_pattern_connected;
	case APP_LED_PAYLOAD_SENT:
		return app_led_pattern_payload_sent;
	case APP_LED_BREATHE:
		return NULL;
	case APP_LED_BREATHE50:
		return NULL;
	case APP_LED_HEARTBEAT:
		return app_led_pattern_heartbeat;
	case APP_LED_CHARGING:
		return NULL;
	case APP_LED_ERROR:
		return app_led_pattern_error;
	case APP_LED_OFF:
	default:
		return NULL;
	}
}

static const char *app_led_pattern_name(enum app_led_pattern pattern)
{
	switch (pattern) {
	case APP_LED_BOOT:
		return "boot";
	case APP_LED_GPS_POWER_ON:
		return "gpson";
	case APP_LED_GPS_ACQUIRING:
		return "gps";
	case APP_LED_GPS_FIX:
		return "fix";
	case APP_LED_MOTION_WAKE:
		return "motion";
	case APP_LED_TRACKING_MOTION:
		return "tracking-motion";
	case APP_LED_CONNECTED:
		return "connected";
	case APP_LED_PAYLOAD_SENT:
		return "payload";
	case APP_LED_BREATHE:
		return "breathe";
	case APP_LED_BREATHE50:
		return "breathe50";
	case APP_LED_HEARTBEAT:
		return "heartbeat";
	case APP_LED_CHARGING:
		return "charging";
	case APP_LED_ERROR:
		return "error";
	case APP_LED_OFF:
	default:
		return "off";
	}
}

static void app_led_work_handler(struct k_work *work)
{
	const struct app_led_step *step;

	ARG_UNUSED(work);

	if (!app_led_ready) {
		app_led_set_brightness(0U);
		return;
	}

	if (app_led_current_pattern == APP_LED_BREATHE ||
	    app_led_current_pattern == APP_LED_BREATHE50 ||
	    app_led_current_pattern == APP_LED_GPS_ACQUIRING) {
		app_led_set_brightness(app_led_breathe_brightness);

		if (app_led_breathe_rising) {
			if (app_led_breathe_brightness >= app_led_breathe_max) {
				app_led_breathe_rising = false;
				app_led_breathe_brightness =
					(app_led_breathe_max > 0U) ? (app_led_breathe_max - 1U) : 0U;
			} else {
				app_led_breathe_brightness++;
			}
		} else {
			if (app_led_breathe_brightness == 0U) {
				app_led_breathe_rising = true;
				app_led_breathe_brightness = 1U;
			} else {
				app_led_breathe_brightness--;
			}
		}

		k_work_reschedule(&app_led_work, K_MSEC(20));
		return;
	}

	if (app_led_steps == NULL) {
		app_led_set_brightness(0U);
		return;
	}

	step = &app_led_steps[app_led_step_index];
	if (step->duration_ms == UINT16_MAX) {
		app_led_step_index = 0;
		step = &app_led_steps[app_led_step_index];
	}

	if (step->duration_ms == 0) {
		app_led_steps = NULL;
		app_led_step_index = 0;
		app_led_set_brightness(0U);
		return;
	}

	app_led_set_brightness(step->brightness_pct);
	app_led_step_index++;
	k_work_reschedule(&app_led_work, K_MSEC(step->duration_ms));
}

static void app_led_apply_pattern(enum app_led_pattern pattern)
{
#if !defined(CONFIG_APP_LED_FEEDBACK)
	ARG_UNUSED(pattern);
	return;
#else
	if (!app_led_ready) {
		return;
	}

	k_work_cancel_delayable(&app_led_work);
	app_led_steps = app_led_pattern_steps(pattern);
	app_led_step_index = 0;
	app_led_current_pattern = pattern;
	LOG_INF("green LED pattern -> %s", app_led_pattern_name(pattern));

	if (pattern == APP_LED_BREATHE) {
		app_led_breathe_brightness = 0U;
		app_led_breathe_max = 100U;
		app_led_breathe_rising = true;
		k_work_reschedule(&app_led_work, K_NO_WAIT);
		return;
	}

	if (pattern == APP_LED_BREATHE50) {
		app_led_breathe_brightness = 0U;
		app_led_breathe_max = 50U;
		app_led_breathe_rising = true;
		k_work_reschedule(&app_led_work, K_NO_WAIT);
		return;
	}

	if (pattern == APP_LED_GPS_ACQUIRING) {
		app_led_breathe_brightness = 8U;
		app_led_breathe_max = 45U;
		app_led_breathe_rising = true;
		k_work_reschedule(&app_led_work, K_NO_WAIT);
		return;
	}

	if (pattern == APP_LED_CHARGING) {
		app_led_set_brightness(1U);
		return;
	}

	if (app_led_steps == NULL) {
		app_led_set_brightness(0U);
		return;
	}

	app_led_work_handler(&app_led_work.work);
#endif
}

static void app_led_pattern(enum app_led_pattern pattern)
{
#if !defined(CONFIG_APP_LED_FEEDBACK)
	ARG_UNUSED(pattern);
	return;
#else
	if (app_led_manual_override) {
		return;
	}

	app_led_apply_pattern(pattern);
#endif
}

static void app_led_force_pattern(enum app_led_pattern pattern)
{
#if !defined(CONFIG_APP_LED_FEEDBACK)
	ARG_UNUSED(pattern);
	return;
#else
	app_led_manual_override = true;
	app_led_apply_pattern(pattern);
#endif
}

static void app_led_manual_brightness(uint8_t brightness_pct)
{
#if !defined(CONFIG_APP_LED_FEEDBACK)
	ARG_UNUSED(brightness_pct);
	return;
#else
	if (!app_led_ready) {
		return;
	}

	app_led_manual_override = true;
	k_work_cancel_delayable(&app_led_work);
	app_led_steps = NULL;
	app_led_step_index = 0;
	app_led_current_pattern = APP_LED_OFF;
	LOG_INF("green LED brightness -> %u%%", brightness_pct);
	app_led_set_brightness(brightness_pct);
#endif
}

static void app_led_release_override(void)
{
#if defined(CONFIG_APP_LED_FEEDBACK)
	app_led_manual_override = false;
	app_led_apply_pattern(APP_LED_OFF);
#endif
}

static void app_led_init(void)
{
	k_work_init_delayable(&app_led_work, app_led_work_handler);

#if defined(CONFIG_APP_LED_FEEDBACK)
	if (!pwm_is_ready_dt(&app_led)) {
		LOG_WRN("green LED PWM not ready");
		return;
	}

	app_led_ready = true;
	app_led_set_brightness(0U);
	LOG_INF("green LED ready on pwm-led0");
#endif
}

static uint32_t app_failure_sleep_seconds(int consecutive_failures)
{
	uint32_t failure_backoff_seconds[] = {
		0U,
		300U,
		600U,
		1800U,
		3600U,
	};
	size_t index;

	if (consecutive_failures <= 0) {
		return app_tracking_interval_seconds;
	}

	failure_backoff_seconds[0] = app_tracking_interval_seconds;

	index = MIN((size_t)(consecutive_failures - 1),
		    ARRAY_SIZE(failure_backoff_seconds) - 1U);

	return failure_backoff_seconds[index];
}

static const char *app_tracker_mode_name(enum tracker_mode mode)
{
	return mode == TRACKER_TRACKING ? "TRACKING" : "PARKED";
}

static char app_tracker_mode_code(enum tracker_mode mode)
{
	return mode == TRACKER_TRACKING ? 'T' : 'P';
}

static char app_status_marker_connect_code(bool heartbeat_only)
{
	if (app_pending_status_marker == 'W') {
		return 'W';
	}

	if (heartbeat_only) {
		return 'H';
	}

	return 'A';
}

static void app_tracker_set_mode(enum tracker_mode new_mode, const char *reason)
{
	enum tracker_mode old_mode = app_tracker_mode;

	if (app_tracker_mode == new_mode) {
		return;
	}

	LOG_INF("mode change: %s -> %s reason=%s uptime=%lld",
		app_tracker_mode_name(app_tracker_mode),
		app_tracker_mode_name(new_mode),
		reason != NULL ? reason : "n/a",
		k_uptime_get());

	app_tracker_mode = new_mode;
	app_consecutive_slow_fixes = 0;
	if (new_mode == TRACKER_PARKED) {
		app_motion_detected = false;
		app_tracking_motion_seen = false;
		app_tracking_motion_events = 0;
	} else if (new_mode == TRACKER_TRACKING) {
		app_tracking_motion_seen = false;
		app_tracking_motion_events = 0;
	}

#if defined(CONFIG_APP_STATUS_MARKERS)
	if (old_mode == TRACKER_PARKED && new_mode == TRACKER_TRACKING) {
		app_pending_status_marker = 'W';
	} else if (old_mode == TRACKER_TRACKING && new_mode == TRACKER_PARKED) {
		app_pending_status_marker = 'P';
	}
#endif

#if defined(CONFIG_APP_LED_MOTION_WAKE)
	if (old_mode == TRACKER_PARKED && new_mode == TRACKER_TRACKING) {
		app_led_pattern(APP_LED_MOTION_WAKE);
	}
#endif
}

static void app_sensor_value_from_mg(int32_t mg, struct sensor_value *val)
{
	int64_t micro_ms2 = ((int64_t)mg * SENSOR_G) / 1000;

	val->val1 = (int32_t)(micro_ms2 / 1000000LL);
	val->val2 = (int32_t)(micro_ms2 % 1000000LL);
}

static void app_motion_trigger_handler(const struct device *dev,
				       const struct sensor_trigger *trig)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(trig);

	if (app_tracker_mode == TRACKER_PARKED && !app_motion_detected) {
		app_motion_detected = true;
		LOG_DBG("motion trigger detected");
	} else if (app_tracker_mode == TRACKER_TRACKING) {
		if (app_tracking_motion_seen) {
			return;
		}

		app_tracking_motion_seen = true;
		app_tracking_motion_events = 1U;
		LOG_INF("tracking interval motion latched count=%u",
			app_tracking_motion_events);
#if defined(CONFIG_APP_LED_MOTION_WAKE)
		app_led_pattern(APP_LED_TRACKING_MOTION);
#endif
	}
}

static int app_motion_trigger_init(void)
{
	struct sensor_value freq = {
		.val1 = 100,
		.val2 = 0,
	};
	struct sensor_value threshold;
	struct sensor_value duration = {
		.val1 = (int32_t)app_motion_duration_samples,
		.val2 = 0,
	};
	struct sensor_value filter_cfg = {
		.val1 = LIS2DH_MOTION_FILTER_CFG,
		.val2 = 0,
	};
	int err;

	if (!device_is_ready(sensor_accel)) {
		return -ENODEV;
	}

	err = sensor_attr_set(sensor_accel, SENSOR_CHAN_ACCEL_XYZ,
			      SENSOR_ATTR_SAMPLING_FREQUENCY, &freq);
	if (err < 0) {
		return err;
	}

	app_sensor_value_from_mg((int32_t)app_motion_threshold_mg, &threshold);
	err = sensor_attr_set(sensor_accel, SENSOR_CHAN_ACCEL_XYZ,
			      SENSOR_ATTR_SLOPE_TH, &threshold);
	if (err < 0) {
		return err;
	}

	err = sensor_attr_set(sensor_accel, SENSOR_CHAN_ACCEL_XYZ,
			      SENSOR_ATTR_SLOPE_DUR, &duration);
	if (err < 0) {
		return err;
	}

	err = sensor_attr_set(sensor_accel, SENSOR_CHAN_ACCEL_XYZ,
			      SENSOR_ATTR_CONFIGURATION, &filter_cfg);
	if (err < 0) {
		return err;
	}

	err = sensor_trigger_set(sensor_accel, &app_motion_trigger,
				 app_motion_trigger_handler);
	if (err < 0) {
		return err;
	}

	app_motion_trigger_ready = true;
	LOG_INF("motion trigger armed threshold=%dmg duration=%d samples filter=0x%x",
		app_motion_threshold_mg,
		app_motion_duration_samples,
		LIS2DH_MOTION_FILTER_CFG);

	return 0;
}

static void app_wait_parked_interval(uint32_t seconds)
{
	LOG_INF("parked wait interval=%us", seconds);

	for (uint32_t elapsed = 0; elapsed < seconds; elapsed++) {
		if (app_force_cycle_requested) {
			app_force_cycle_requested = false;
			LOG_INF("manual cycle request consumed during parked wait");
			return;
		}

		if (app_tracker_mode != TRACKER_PARKED) {
			return;
		}

		if (app_motion_detected) {
			app_motion_detected = false;
			app_tracker_set_mode(TRACKER_TRACKING, "motion");
			return;
		}

		app_wdt_feed();
		k_sleep(K_SECONDS(1));
	}
}

static void app_wait_tracking_interval(uint32_t seconds)
{
	LOG_INF("tracking wait interval=%us", seconds);

	for (uint32_t elapsed = 0; elapsed < seconds; elapsed++) {
		if (app_force_cycle_requested) {
			app_force_cycle_requested = false;
			LOG_INF("manual cycle request consumed during tracking wait");
			return;
		}

		app_wdt_feed();
		k_sleep(K_SECONDS(1));
	}
}

static bool app_manual_cycle_mode_enabled(void)
{
	return IS_ENABLED(CONFIG_APP_DEBUG_MANUAL_CYCLE_MODE);
}

static void app_wait_manual_cycle_request(const char *context)
{
	int ret;

	if (!app_manual_cycle_mode_enabled()) {
		return;
	}

	if (!app_manual_cycle_idle_announced) {
		LOG_INF("manual-cycle debug mode idle (%s); waiting for runcycle/runcycle_nopsm",
			context);
		app_manual_cycle_idle_announced = true;
		if (!app_debug_resumetracker_autotest_scheduled) {
			app_debug_resumetracker_autotest_scheduled = true;
			LOG_INF("debug autorun: scheduling delayed inline connect->reopen->tracker self-test");
			k_work_schedule_for_queue(&app_debug_workq,
						 &app_debug_sendtracker_autorun_work,
						 K_SECONDS(20));
		}
	}

	while (!app_force_cycle_requested) {
		app_wdt_feed();
		k_sleep(K_SECONDS(1));
	}

	app_manual_cycle_idle_announced = false;
}

static void app_sleep_after_failure(struct golioth_client *client, int counter,
				       uint32_t next_sleep_seconds,
				       uint32_t next_wake_seconds,
				       const char *reason)
{
	golioth_client_stop(client);
	if (!app_manual_cycle_mode_enabled() && !app_cycle_no_psm_active &&
	    app_psm_disabled == 0U) {
		int psm_ret = bg9x_psm_set_interval(true, next_wake_seconds);

		if (psm_ret < 0) {
			LOG_WRN("failure-sleep PSM re-enable failed: %d", psm_ret);
		}
	}
#if defined(CONFIG_APP_LED_ERROR)
	app_led_pattern(APP_LED_ERROR);
#else
	app_led_pattern(APP_LED_OFF);
#endif
	if (app_tracker_mode == TRACKER_TRACKING) {
		LOG_WRN("#%d %s, tracking failure sleep %us", counter, reason,
			next_sleep_seconds);
	} else {
		LOG_WRN("#%d %s, parked failure sleep %us", counter, reason,
			next_sleep_seconds);
	}
	app_wdt_feed();
	if (app_manual_cycle_mode_enabled()) {
		LOG_WRN("#%d %s, debug manual-cycle mode staying awake", counter, reason);
		app_cycle_no_psm_active = false;
		app_wait_manual_cycle_request("failure-idle");
		return;
	}
	if (app_tracker_mode == TRACKER_TRACKING) {
		app_wait_tracking_interval(next_sleep_seconds);
	} else {
		app_wait_parked_interval(next_sleep_seconds);
	}
}

static void app_tracker_note_fix(const struct bg9x_gps_fix *fix)
{
	int32_t speed_x10;
	bool motion_seen;
	uint32_t motion_events;

	if (app_tracker_mode != TRACKER_TRACKING || fix == NULL || !fix->valid) {
		return;
	}

	motion_seen = app_tracking_motion_seen;
	motion_events = app_tracking_motion_events;
	app_tracking_motion_seen = false;
	app_tracking_motion_events = 0;
	app_tracking_motion_events_last_interval = motion_events;

	speed_x10 = (int32_t)(fix->speed_kmh * 10.0f);
	if (motion_seen) {
		app_consecutive_slow_fixes = 0;
		LOG_INF("tracking interval motion seen count=%u; slow-fix count reset speed=%.1f km/h",
			motion_events, (double)fix->speed_kmh);
	} else if (speed_x10 < ((int32_t)app_tracking_speed_threshold_kmh * 10)) {
		app_consecutive_slow_fixes++;
		LOG_INF("tracking slow-fix count=%d/%d speed=%.1f km/h motion=0",
			app_consecutive_slow_fixes,
			app_tracking_stop_count,
			(double)fix->speed_kmh);
		if (app_consecutive_slow_fixes >= (int)app_tracking_stop_count) {
			app_tracker_set_mode(TRACKER_PARKED, "speed-below-threshold-no-motion");
		}
	} else {
		app_consecutive_slow_fixes = 0;
		LOG_INF("tracking motion/speed keeps tracking speed=%.1f km/h motion=0",
			(double)fix->speed_kmh);
	}
}

static void app_wdt_feed(void)
{
	int err;

	if (app_wdt_channel_id < 0) {
		return;
	}

	err = wdt_feed(app_wdt, app_wdt_channel_id);
	if (err < 0) {
		LOG_WRN("watchdog feed failed: %d", err);
	}
}

static void app_wdt_init(void)
{
	struct wdt_timeout_cfg wdt_config = {
		.flags = WDT_FLAG_RESET_SOC,
		.window.min = 0U,
		.window.max = APP_WDT_TIMEOUT_SECONDS * MSEC_PER_SEC,
	};
	int err;

	if (!device_is_ready(app_wdt)) {
		LOG_WRN("watchdog device not ready");
		return;
	}

	app_wdt_channel_id = wdt_install_timeout(app_wdt, &wdt_config);
	if (app_wdt_channel_id < 0) {
		LOG_WRN("watchdog install failed: %d", app_wdt_channel_id);
		app_wdt_channel_id = -1;
		return;
	}

	err = wdt_setup(app_wdt, WDT_OPT_PAUSE_HALTED_BY_DBG);
	if (err < 0) {
		LOG_WRN("watchdog setup failed: %d", err);
		app_wdt_channel_id = -1;
		return;
	}

	LOG_INF("watchdog enabled timeout=%ds", APP_WDT_TIMEOUT_SECONDS);
	app_wdt_feed();
}

static int app_settings_set(const char *name, size_t len_rd,
			    settings_read_cb read_cb, void *cb_arg)
{
	uint32_t val;
	ssize_t len;

	len = read_cb(cb_arg, &val, sizeof(val));
	if (len != sizeof(val)) {
		return -EINVAL;
	}

	if (!strcmp(name, "boot_count")) {
		app_boot_count = val;
	} else if (!strcmp(name, "cycle_count")) {
		app_total_cycles = val;
	} else if (!strcmp(name, "bytes_today")) {
		app_bytes_today = val;
	} else if (!strcmp(name, "day_boundary")) {
		app_day_boundary = val;
		app_day_boundary_valid = (val != 0U);
	} else if (!strcmp(name, "psm_disable")) {
		app_psm_disabled = val;
	} else if (!strcmp(name, "gps_enable")) {
		app_gps_enabled = val;
	} else if (!strcmp(name, "tracking_interval_s")) {
		app_tracking_interval_seconds = val;
	} else if (!strcmp(name, "parked_interval_s")) {
		app_parked_interval_seconds = val;
	} else if (!strcmp(name, "motion_threshold_mg")) {
		app_motion_threshold_mg = val;
	} else if (!strcmp(name, "motion_duration_samples")) {
		app_motion_duration_samples = val;
	} else if (!strcmp(name, "tracking_speed_threshold_kmh")) {
		app_tracking_speed_threshold_kmh = val;
	} else if (!strcmp(name, "tracking_stop_count")) {
		app_tracking_stop_count = val;
	} else if (!strcmp(name, "gps_timeout_seconds")) {
		app_gps_timeout_seconds = val;
	} else if (!strcmp(name, "cellular_timeout_seconds")) {
		app_cellular_timeout_seconds = val;
	} else if (!strcmp(name, "wake_anchored_intervals")) {
		app_wake_anchored_intervals = val;
	} else if (!strcmp(name, "cbor_enabled")) {
		app_cbor_enabled = val;
	} else if (!strcmp(name, "minified_payload")) {
		app_minified_payload = val;
	} else if (!strcmp(name, "xtra_enabled")) {
		app_xtra_enabled = val;
	} else if (!strcmp(name, "xtra_retention")) {
		app_xtra_retention_days = val;
	} else if (!strcmp(name, "xtra_mode")) {
		app_xtra_mode = val;
	} else if (!strcmp(name, "gps_lat")) {
		app_gps_lat_e6 = (int32_t)val;
	} else if (!strcmp(name, "gps_lon")) {
		app_gps_lon_e6 = (int32_t)val;
	} else if (!strcmp(name, "gps_alt")) {
		app_gps_alt_cm = (int32_t)val;
	} else if (!strcmp(name, "gps_speed")) {
		app_gps_speed_x10 = val;
	} else if (!strcmp(name, "gps_course")) {
		app_gps_course_x10 = val;
	} else if (!strcmp(name, "gps_sats")) {
		app_gps_sats = val;
	} else if (!strcmp(name, "gps_fixtime")) {
		app_gps_fixtime_ms = val;
	} else if (!strcmp(name, "gps_uptime")) {
		app_gps_uptime_ms = val;
	} else {
		return -ENOENT;
	}

	return 0;
}

static int app_settings_get(const char *name, char *dst, int val_len_max)
{
	uint32_t val;

	if (!strcmp(name, "boot_count")) {
		val = app_boot_count;
	} else if (!strcmp(name, "cycle_count")) {
		val = app_total_cycles;
	} else if (!strcmp(name, "bytes_today")) {
		val = app_bytes_today;
	} else if (!strcmp(name, "day_boundary")) {
		val = app_day_boundary;
	} else if (!strcmp(name, "psm_disable")) {
		val = app_psm_disabled;
	} else if (!strcmp(name, "gps_enable")) {
		val = app_gps_enabled;
	} else if (!strcmp(name, "tracking_interval_s")) {
		val = app_tracking_interval_seconds;
	} else if (!strcmp(name, "parked_interval_s")) {
		val = app_parked_interval_seconds;
	} else if (!strcmp(name, "motion_threshold_mg")) {
		val = app_motion_threshold_mg;
	} else if (!strcmp(name, "motion_duration_samples")) {
		val = app_motion_duration_samples;
	} else if (!strcmp(name, "tracking_speed_threshold_kmh")) {
		val = app_tracking_speed_threshold_kmh;
	} else if (!strcmp(name, "tracking_stop_count")) {
		val = app_tracking_stop_count;
	} else if (!strcmp(name, "gps_timeout_seconds")) {
		val = app_gps_timeout_seconds;
	} else if (!strcmp(name, "cellular_timeout_seconds")) {
		val = app_cellular_timeout_seconds;
	} else if (!strcmp(name, "wake_anchored_intervals")) {
		val = app_wake_anchored_intervals;
	} else if (!strcmp(name, "cbor_enabled")) {
		val = app_cbor_enabled;
	} else if (!strcmp(name, "minified_payload")) {
		val = app_minified_payload;
	} else if (!strcmp(name, "xtra_enabled")) {
		val = app_xtra_enabled;
	} else if (!strcmp(name, "xtra_retention")) {
		val = app_xtra_retention_days;
	} else if (!strcmp(name, "xtra_mode")) {
		val = app_xtra_mode;
	} else if (!strcmp(name, "gps_lat")) {
		val = (uint32_t)app_gps_lat_e6;
	} else if (!strcmp(name, "gps_lon")) {
		val = (uint32_t)app_gps_lon_e6;
	} else if (!strcmp(name, "gps_alt")) {
		val = (uint32_t)app_gps_alt_cm;
	} else if (!strcmp(name, "gps_speed")) {
		val = app_gps_speed_x10;
	} else if (!strcmp(name, "gps_course")) {
		val = app_gps_course_x10;
	} else if (!strcmp(name, "gps_sats")) {
		val = app_gps_sats;
	} else if (!strcmp(name, "gps_fixtime")) {
		val = app_gps_fixtime_ms;
	} else if (!strcmp(name, "gps_uptime")) {
		val = app_gps_uptime_ms;
	} else {
		return -ENOENT;
	}

	if (val_len_max < (int)sizeof(val)) {
		return -ENOMEM;
	}

	memcpy(dst, &val, sizeof(val));
	return sizeof(val);
}

SETTINGS_STATIC_HANDLER_DEFINE(app, "app",
			       IS_ENABLED(CONFIG_SETTINGS_RUNTIME) ? app_settings_get : NULL,
			       app_settings_set, NULL, NULL);

static void app_settings_init(void)
{
	int err;

	k_mutex_init(&app_settings_cb_lock);
	app_debug_workq_init();
	k_work_init_delayable(&app_debug_sendtracker_autorun_work,
			      app_debug_sendtracker_autorun_work_handler);
	k_work_init(&app_debug_sendtracker_work, app_debug_sendtracker_work_handler);
	k_work_init(&app_debug_resumetracker_work, app_debug_resumetracker_work_handler);
	app_day_boundary_valid = (app_day_boundary != 0U);

	app_boot_count++;
	err = settings_save_one("app/boot_count", &app_boot_count, sizeof(app_boot_count));
	if (err < 0) {
		LOG_WRN("settings boot_count save failed: %d", err);
	}

	LOG_INF("settings ready boot=%u total_cycle=%u bytes_today=%u day_boundary=%u psm_dis=%u",
		app_boot_count, app_total_cycles, app_bytes_today, app_day_boundary,
		app_psm_disabled);
	LOG_INF("build tag=%s", APP_DEBUG_BUILD_TAG);
	LOG_INF("config tracking=%us parked=%us motion=%umg/%u speed=%ukmh stop=%u gps=%u psm=%u cbor=%u min=%u",
		app_tracking_interval_seconds, app_parked_interval_seconds,
		app_motion_threshold_mg, app_motion_duration_samples,
		app_tracking_speed_threshold_kmh, app_tracking_stop_count,
		app_gps_enabled, app_psm_disabled ? 0U : 1U,
		app_cbor_enabled, app_minified_payload);
	LOG_INF("config gps_timeout=%us cellular_timeout=%us wake_anchored=%u xtra=%u/%u/%u",
		app_gps_timeout_seconds,
		app_cellular_timeout_seconds,
		app_wake_anchored_intervals,
		app_xtra_enabled,
		app_xtra_retention_days,
		app_xtra_mode);

	if (app_gps_lat_e6 != 0 || app_gps_lon_e6 != 0) {
		uint32_t age_ms = (uint32_t)k_uptime_get() > app_gps_uptime_ms ?
			0 : (uint32_t)k_uptime_get();
		LOG_INF("last GPS: lat=%d.%06d lon=%d.%06d sats=%u fixtime=%ums",
			app_gps_lat_e6 / 1000000, abs(app_gps_lat_e6) % 1000000,
			app_gps_lon_e6 / 1000000, abs(app_gps_lon_e6) % 1000000,
			app_gps_sats, app_gps_fixtime_ms);
		ARG_UNUSED(age_ms);
	}
}

static void app_cycle_counter_advance(void)
{
	int err;

	app_total_cycles++;
	err = settings_save_one("app/cycle_count", &app_total_cycles, sizeof(app_total_cycles));
	if (err < 0) {
		LOG_WRN("settings cycle_count save failed: %d", err);
	}
}

static void app_cycle_tx_bytes_reset(void)
{
	app_cycle_tx_bytes = 0U;
	app_cycle_aux_tx_bytes = 0U;
	app_cycle_tracker_tx_bytes = 0U;
}

static void app_bytes_today_commit(void)
{
	int err;

	err = settings_save_one("app/bytes_today", &app_bytes_today, sizeof(app_bytes_today));
	if (err < 0) {
		LOG_WRN("settings bytes_today save failed: %d", err);
	}

	err = settings_save_one("app/day_boundary", &app_day_boundary, sizeof(app_day_boundary));
	if (err < 0) {
		LOG_WRN("settings day_boundary save failed: %d", err);
	}
}

static uint32_t app_lightdb_int_payload_bytes(const char *path, int32_t value)
{
	char value_buf[16];
	int value_len;

	value_len = snprintk(value_buf, sizeof(value_buf), "%d", value);
	if (value_len < 0) {
		return 0U;
	}

	return (uint32_t)(strlen(path) + value_len);
}

static uint32_t app_lightdb_string_payload_bytes(const char *path, size_t value_len)
{
	return (uint32_t)(strlen(path) + value_len);
}

static void app_cycle_tx_bytes_add(uint32_t bytes)
{
	app_cycle_tx_bytes += bytes;
	app_cycle_aux_tx_bytes += bytes;
}

static void app_cycle_tracker_tx_bytes_add(uint32_t bytes)
{
	app_cycle_tx_bytes += bytes;
	app_cycle_tracker_tx_bytes += bytes;
}

static void app_record_tracker_payload(const char *path, size_t payload_len, const char *payload_text)
{
	app_last_tracker_payload_len = (uint32_t)payload_len;
	app_last_tracker_path_len = (uint32_t)strlen(path);
	app_last_tracker_tx_estimate = app_last_tracker_path_len + app_last_tracker_payload_len;
	snprintk(app_last_tracker_path, sizeof(app_last_tracker_path), "%s", path);
#if defined(CONFIG_APP_LOG_RAW_TRACKER_PAYLOAD)
	snprintk(app_last_tracker_payload, sizeof(app_last_tracker_payload), "%s",
		 payload_text != NULL ? payload_text : "");
#else
	ARG_UNUSED(payload_text);
#endif
}

static void app_log_tracker_payload_debug(const char *content_label, bool minified)
{
	printk("tracker tx calc content=%s path=%s payload=%u path=%u est=%u min=%u\n",
	       content_label,
	       app_last_tracker_path,
	       app_last_tracker_payload_len,
	       app_last_tracker_path_len,
	       app_last_tracker_tx_estimate,
	       minified ? 1U : 0U);
#if defined(CONFIG_APP_LOG_RAW_TRACKER_PAYLOAD)
	printk("tracker payload %s\n", app_last_tracker_payload);
#endif
}

static enum golioth_status publish_lightdb_int_state(const char *path, int32_t value)
{
	enum golioth_status status;

	status = golioth_lightdb_set_int_sync(client, path, value, APP_STATE_TIMEOUT_SECONDS);
	if (status == GOLIOTH_OK) {
		app_cycle_tx_bytes_add(app_lightdb_int_payload_bytes(path, value));
	}

	return status;
}

static enum golioth_status publish_lightdb_string_state(const char *path, const char *value)
{
	enum golioth_status status;
	size_t value_len;

	if (value == NULL) {
		return GOLIOTH_ERR_INVALID_FORMAT;
	}

	value_len = strlen(value);
	status = golioth_lightdb_set_string_sync(client, path, value, value_len,
						 APP_STATE_TIMEOUT_SECONDS);
	if (status == GOLIOTH_OK) {
		app_cycle_tx_bytes_add(app_lightdb_string_payload_bytes(path, value_len));
	}

	return status;
}

static bool app_path_join(char *buf, size_t buf_size, const char *root, const char *leaf)
{
	int len;

	if (buf == NULL || buf_size == 0 || root == NULL || leaf == NULL) {
		return false;
	}

	len = snprintk(buf, buf_size, "%s/%s", root, leaf);
	return len > 0 && (size_t)len < buf_size;
}

static enum golioth_status publish_lightdb_json_state(const char *path, const char *json)
{
	enum golioth_status status;
	size_t json_len;

	if (path == NULL || json == NULL) {
		return GOLIOTH_ERR_INVALID_FORMAT;
	}

	json_len = strlen(json);
	status = golioth_lightdb_set_sync(client, path, GOLIOTH_CONTENT_TYPE_JSON,
					  (const uint8_t *)json, json_len,
					  APP_STATE_TIMEOUT_SECONDS);
	if (status == GOLIOTH_OK) {
		app_cycle_tx_bytes_add(app_lightdb_string_payload_bytes(path, json_len));
	}

	return status;
}

static enum golioth_status publish_tracker_int_state(const char *path, int32_t value)
{
	char full_path[APP_STATE_PATH_MAX];

	if (!app_path_join(full_path, sizeof(full_path), APP_TRACKER_STATE_ROOT, path)) {
		return GOLIOTH_ERR_INVALID_FORMAT;
	}

	return publish_lightdb_int_state(full_path, value);
}

static enum golioth_status publish_tracker_string_state(const char *path, const char *value)
{
	char full_path[APP_STATE_PATH_MAX];

	if (!app_path_join(full_path, sizeof(full_path), APP_TRACKER_STATE_ROOT, path)) {
		return GOLIOTH_ERR_INVALID_FORMAT;
	}

	return publish_lightdb_string_state(full_path, value);
}

static void publish_runtime_config_state_if_dirty(void)
{
	static const char *const legacy_state_keys[] = {
		"cfg",
		"tracker",
		"cfg_tracking_interval_s",
		"cfg_parked_interval_s",
		"cfg_motion_threshold_mg",
		"cfg_motion_duration_samples",
		"cfg_tracking_speed_threshold_kmh",
		"cfg_tracking_stop_count",
		"cfg_wake_anchored_intervals",
		"cfg_gps_enabled",
		"cfg_psm_enabled",
		"tracker",
	};
	bool all_pruned = true;

	if (client == NULL || !app_runtime_cfg_dirty) {
		return;
	}

	if (app_legacy_cfg_state_pruned) {
		app_runtime_cfg_dirty = false;
		return;
	}

	for (size_t i = 0; i < ARRAY_SIZE(legacy_state_keys); i++) {
		enum golioth_status status = delete_lightdb_state(legacy_state_keys[i]);

		if (status != GOLIOTH_OK) {
			all_pruned = false;
			LOG_WRN("legacy state prune pending key=%s status=%s",
				legacy_state_keys[i], golioth_status_to_str(status));
		}
	}

	if (!all_pruned) {
		return;
	}

	app_legacy_cfg_state_pruned = true;
	app_runtime_cfg_dirty = false;
	LOG_INF("legacy state prune complete");
}

static int app_save_u32_setting(const char *name, uint32_t value)
{
	int err = settings_save_one(name, &value, sizeof(value));

	if (err < 0) {
		LOG_WRN("settings save failed %s: %d", name, err);
	}

	return err;
}

static bool app_setting_u32_unchanged(uint32_t current_value, uint32_t new_value)
{
	return current_value == new_value;
}

static bool app_setting_bool_unchanged(uint32_t current_value, bool new_value)
{
	return current_value == (new_value ? 1U : 0U);
}

static bool app_setting_u32_duplicate(struct app_setting_u32_cache *cache, uint32_t new_value)
{
	if (cache->valid && cache->value == new_value) {
		return true;
	}

	cache->valid = true;
	cache->value = new_value;
	return false;
}

static bool app_setting_bool_duplicate(struct app_setting_bool_cache *cache, bool new_value)
{
	if (cache->valid && cache->value == new_value) {
		return true;
	}

	cache->valid = true;
	cache->value = new_value;
	return false;
}

static enum golioth_settings_status on_tracking_interval_setting(int32_t new_value, void *arg)
{
	enum golioth_settings_status status = GOLIOTH_SETTINGS_SUCCESS;

	ARG_UNUSED(arg);
	app_note_settings_activity();
	k_mutex_lock(&app_settings_cb_lock, K_FOREVER);
	if (app_setting_u32_duplicate(&app_cloud_tracking_interval_cache,
				      (uint32_t)new_value)) {
		goto out;
	}
	if (app_setting_u32_unchanged(app_tracking_interval_seconds, (uint32_t)new_value)) {
		goto out;
	}
	app_tracking_interval_seconds = (uint32_t)new_value;
	(void)app_save_u32_setting("app/tracking_interval_s", app_tracking_interval_seconds);
	app_runtime_cfg_dirty = true;
	LOG_INF("settings TRACKING_INTERVAL_S=%u", app_tracking_interval_seconds);
out:
	k_mutex_unlock(&app_settings_cb_lock);
	return status;
}

static enum golioth_settings_status on_parked_interval_setting(int32_t new_value, void *arg)
{
	enum golioth_settings_status status = GOLIOTH_SETTINGS_SUCCESS;

	ARG_UNUSED(arg);
	app_note_settings_activity();
	k_mutex_lock(&app_settings_cb_lock, K_FOREVER);
	if (app_setting_u32_duplicate(&app_cloud_parked_interval_cache,
				      (uint32_t)new_value)) {
		goto out;
	}
	if (app_setting_u32_unchanged(app_parked_interval_seconds, (uint32_t)new_value)) {
		goto out;
	}
	app_parked_interval_seconds = (uint32_t)new_value;
	(void)app_save_u32_setting("app/parked_interval_s", app_parked_interval_seconds);
	app_runtime_cfg_dirty = true;
	LOG_INF("settings PARKED_INTERVAL_S=%u", app_parked_interval_seconds);
out:
	k_mutex_unlock(&app_settings_cb_lock);
	return status;
}

static enum golioth_settings_status on_motion_threshold_setting(int32_t new_value, void *arg)
{
	enum golioth_settings_status status = GOLIOTH_SETTINGS_SUCCESS;

	ARG_UNUSED(arg);
	app_note_settings_activity();
	k_mutex_lock(&app_settings_cb_lock, K_FOREVER);
	if (app_setting_u32_duplicate(&app_cloud_motion_threshold_cache,
				      (uint32_t)new_value)) {
		goto out;
	}
	if (app_setting_u32_unchanged(app_motion_threshold_mg, (uint32_t)new_value)) {
		goto out;
	}
	app_motion_threshold_mg = (uint32_t)new_value;
	(void)app_save_u32_setting("app/motion_threshold_mg", app_motion_threshold_mg);
	if (app_motion_trigger_init() < 0) {
		status = GOLIOTH_SETTINGS_GENERAL_ERROR;
		goto out;
	}
	app_runtime_cfg_dirty = true;
	LOG_INF("settings MOTION_THRESHOLD_MG=%u", app_motion_threshold_mg);
out:
	k_mutex_unlock(&app_settings_cb_lock);
	return status;
}

static enum golioth_settings_status on_motion_duration_setting(int32_t new_value, void *arg)
{
	enum golioth_settings_status status = GOLIOTH_SETTINGS_SUCCESS;

	ARG_UNUSED(arg);
	app_note_settings_activity();
	k_mutex_lock(&app_settings_cb_lock, K_FOREVER);
	if (app_setting_u32_duplicate(&app_cloud_motion_duration_cache,
				      (uint32_t)new_value)) {
		goto out;
	}
	if (app_setting_u32_unchanged(app_motion_duration_samples, (uint32_t)new_value)) {
		goto out;
	}
	app_motion_duration_samples = (uint32_t)new_value;
	(void)app_save_u32_setting("app/motion_duration_samples", app_motion_duration_samples);
	if (app_motion_trigger_init() < 0) {
		status = GOLIOTH_SETTINGS_GENERAL_ERROR;
		goto out;
	}
	app_runtime_cfg_dirty = true;
	LOG_INF("settings MOTION_DURATION_SAMPLES=%u", app_motion_duration_samples);
out:
	k_mutex_unlock(&app_settings_cb_lock);
	return status;
}

static enum golioth_settings_status on_tracking_speed_threshold_setting(int32_t new_value, void *arg)
{
	enum golioth_settings_status status = GOLIOTH_SETTINGS_SUCCESS;

	ARG_UNUSED(arg);
	app_note_settings_activity();
	k_mutex_lock(&app_settings_cb_lock, K_FOREVER);
	if (app_setting_u32_duplicate(&app_cloud_tracking_speed_cache,
				      (uint32_t)new_value)) {
		goto out;
	}
	if (app_setting_u32_unchanged(app_tracking_speed_threshold_kmh, (uint32_t)new_value)) {
		goto out;
	}
	app_tracking_speed_threshold_kmh = (uint32_t)new_value;
	(void)app_save_u32_setting("app/tracking_speed_threshold_kmh",
				      app_tracking_speed_threshold_kmh);
	app_runtime_cfg_dirty = true;
	LOG_INF("settings TRACKING_SPEED_THRESHOLD_KMH=%u", app_tracking_speed_threshold_kmh);
out:
	k_mutex_unlock(&app_settings_cb_lock);
	return status;
}

static enum golioth_settings_status on_tracking_stop_count_setting(int32_t new_value, void *arg)
{
	enum golioth_settings_status status = GOLIOTH_SETTINGS_SUCCESS;

	ARG_UNUSED(arg);
	app_note_settings_activity();
	k_mutex_lock(&app_settings_cb_lock, K_FOREVER);
	if (app_setting_u32_duplicate(&app_cloud_tracking_stop_cache,
				      (uint32_t)new_value)) {
		goto out;
	}
	if (app_setting_u32_unchanged(app_tracking_stop_count, (uint32_t)new_value)) {
		goto out;
	}
	app_tracking_stop_count = (uint32_t)new_value;
	(void)app_save_u32_setting("app/tracking_stop_count", app_tracking_stop_count);
	app_runtime_cfg_dirty = true;
	LOG_INF("settings TRACKING_STOP_COUNT=%u", app_tracking_stop_count);
out:
	k_mutex_unlock(&app_settings_cb_lock);
	return status;
}

static enum golioth_settings_status on_gps_timeout_setting(int32_t new_value, void *arg)
{
	enum golioth_settings_status status = GOLIOTH_SETTINGS_SUCCESS;

	ARG_UNUSED(arg);
	app_note_settings_activity();
	k_mutex_lock(&app_settings_cb_lock, K_FOREVER);
	if (app_setting_u32_duplicate(&app_cloud_gps_timeout_cache, (uint32_t)new_value)) {
		goto out;
	}
	if (app_setting_u32_unchanged(app_gps_timeout_seconds, (uint32_t)new_value)) {
		goto out;
	}
	app_gps_timeout_seconds = (uint32_t)new_value;
	(void)app_save_u32_setting("app/gps_timeout_seconds", app_gps_timeout_seconds);
	app_runtime_cfg_dirty = true;
	LOG_INF("settings GPS_TIMEOUT=%u", app_gps_timeout_seconds);
out:
	k_mutex_unlock(&app_settings_cb_lock);
	return status;
}

static enum golioth_settings_status on_cellular_timeout_setting(int32_t new_value, void *arg)
{
	enum golioth_settings_status status = GOLIOTH_SETTINGS_SUCCESS;

	ARG_UNUSED(arg);
	app_note_settings_activity();
	k_mutex_lock(&app_settings_cb_lock, K_FOREVER);
	if (app_setting_u32_duplicate(&app_cloud_cellular_timeout_cache, (uint32_t)new_value)) {
		goto out;
	}
	if (app_setting_u32_unchanged(app_cellular_timeout_seconds, (uint32_t)new_value)) {
		goto out;
	}
	app_cellular_timeout_seconds = (uint32_t)new_value;
	(void)app_save_u32_setting("app/cellular_timeout_seconds", app_cellular_timeout_seconds);
	app_runtime_cfg_dirty = true;
	LOG_INF("settings CELLULAR_TIMEOUT=%u", app_cellular_timeout_seconds);
out:
	k_mutex_unlock(&app_settings_cb_lock);
	return status;
}

static enum golioth_settings_status on_wake_anchored_intervals_setting(bool new_value, void *arg)
{
	enum golioth_settings_status status = GOLIOTH_SETTINGS_SUCCESS;

	ARG_UNUSED(arg);
	app_note_settings_activity();
	k_mutex_lock(&app_settings_cb_lock, K_FOREVER);
	if (app_setting_bool_duplicate(&app_cloud_wake_anchored_cache, new_value)) {
		goto out;
	}
	if (app_setting_bool_unchanged(app_wake_anchored_intervals, new_value)) {
		goto out;
	}
	app_wake_anchored_intervals = new_value ? 1U : 0U;
	(void)app_save_u32_setting("app/wake_anchored_intervals", app_wake_anchored_intervals);
	app_runtime_cfg_dirty = true;
	LOG_INF("settings WAKE_ANCHORED_INTERVALS=%u", app_wake_anchored_intervals);
out:
	k_mutex_unlock(&app_settings_cb_lock);
	return status;
}

static enum golioth_settings_status on_gps_enabled_setting(bool new_value, void *arg)
{
	enum golioth_settings_status status = GOLIOTH_SETTINGS_SUCCESS;

	ARG_UNUSED(arg);
	app_note_settings_activity();
	k_mutex_lock(&app_settings_cb_lock, K_FOREVER);
	if (app_setting_bool_duplicate(&app_cloud_gps_enabled_cache, new_value)) {
		goto out;
	}
	if (app_setting_bool_unchanged(app_gps_enabled, new_value)) {
		goto out;
	}
	app_gps_enabled = new_value ? 1U : 0U;
	(void)app_save_u32_setting("app/gps_enable", app_gps_enabled);
	app_runtime_cfg_dirty = true;
	LOG_INF("settings GPS_ENABLED=%u", app_gps_enabled);
out:
	k_mutex_unlock(&app_settings_cb_lock);
	return status;
}

static enum golioth_settings_status on_psm_enabled_setting(bool new_value, void *arg)
{
	enum golioth_settings_status status = GOLIOTH_SETTINGS_SUCCESS;

	ARG_UNUSED(arg);
	app_note_settings_activity();
	k_mutex_lock(&app_settings_cb_lock, K_FOREVER);
	if (app_setting_bool_duplicate(&app_cloud_psm_enabled_cache, new_value)) {
		goto out;
	}
	if (app_setting_bool_unchanged(app_psm_disabled ? 0U : 1U, new_value)) {
		goto out;
	}
	app_psm_disabled = new_value ? 0U : 1U;
	(void)app_save_u32_setting("app/psm_disable", app_psm_disabled);
	if (!new_value) {
		(void)bg9x_psm_set(false);
	}
	app_runtime_cfg_dirty = true;
	LOG_INF("settings PSM_ENABLED=%u", new_value ? 1U : 0U);
out:
	k_mutex_unlock(&app_settings_cb_lock);
	return status;
}

static enum golioth_settings_status on_cbor_enabled_setting(bool new_value, void *arg)
{
	enum golioth_settings_status status = GOLIOTH_SETTINGS_SUCCESS;

	ARG_UNUSED(arg);
	app_note_settings_activity();
	k_mutex_lock(&app_settings_cb_lock, K_FOREVER);
	if (app_setting_bool_duplicate(&app_cloud_cbor_enabled_cache, new_value)) {
		goto out;
	}
	if (app_setting_bool_unchanged(app_cbor_enabled, new_value)) {
		goto out;
	}
	app_cbor_enabled = new_value ? 1U : 0U;
	(void)app_save_u32_setting("app/cbor_enabled", app_cbor_enabled);
	app_runtime_cfg_dirty = true;
	LOG_INF("settings CBOR_ENABLED=%u", app_cbor_enabled);
out:
	k_mutex_unlock(&app_settings_cb_lock);
	return status;
}

static enum golioth_settings_status on_minified_payload_setting(bool new_value, void *arg)
{
	enum golioth_settings_status status = GOLIOTH_SETTINGS_SUCCESS;

	ARG_UNUSED(arg);
	app_note_settings_activity();
	k_mutex_lock(&app_settings_cb_lock, K_FOREVER);
	if (app_setting_bool_duplicate(&app_cloud_minified_payload_cache, new_value)) {
		goto out;
	}
	if (app_setting_bool_unchanged(app_minified_payload, new_value)) {
		goto out;
	}
	app_minified_payload = new_value ? 1U : 0U;
	(void)app_save_u32_setting("app/minified_payload", app_minified_payload);
	app_runtime_cfg_dirty = true;
	LOG_INF("settings MINIFIED_PAYLOAD=%u", app_minified_payload);
out:
	k_mutex_unlock(&app_settings_cb_lock);
	return status;
}

static enum golioth_settings_status on_xtra_enabled_setting(bool new_value, void *arg)
{
	int ret;
	enum golioth_settings_status status = GOLIOTH_SETTINGS_SUCCESS;

	ARG_UNUSED(arg);
	app_note_settings_activity();
	k_mutex_lock(&app_settings_cb_lock, K_FOREVER);
	if (app_setting_bool_duplicate(&app_cloud_xtra_enabled_cache, new_value)) {
		goto out;
	}
	if (app_setting_bool_unchanged(app_xtra_enabled, new_value)) {
		goto out;
	}

	app_xtra_enabled = new_value ? 1U : 0U;
	(void)app_save_u32_setting("app/xtra_enabled", app_xtra_enabled);
	ret = bg9x_gps_xtra_policy_set(app_xtra_enabled != 0U,
				       (uint8_t)app_xtra_retention_days,
				       (uint8_t)app_xtra_mode);
	if (ret < 0) {
		status = GOLIOTH_SETTINGS_GENERAL_ERROR;
		goto out;
	}
	app_runtime_cfg_dirty = true;
	LOG_INF("settings XTRA_ENABLED=%u", app_xtra_enabled);
out:
	k_mutex_unlock(&app_settings_cb_lock);
	return status;
}

static enum golioth_settings_status on_xtra_retention_setting(int32_t new_value, void *arg)
{
	int ret;
	uint32_t value = (uint32_t)new_value;
	enum golioth_settings_status status = GOLIOTH_SETTINGS_SUCCESS;

	ARG_UNUSED(arg);
	app_note_settings_activity();
	k_mutex_lock(&app_settings_cb_lock, K_FOREVER);
	if (app_setting_u32_duplicate(&app_cloud_xtra_retention_cache, value)) {
		goto out;
	}
	if (value != 1U && value != 2U && value != 3U && value != 7U) {
		status = GOLIOTH_SETTINGS_GENERAL_ERROR;
		goto out;
	}
	if (app_setting_u32_unchanged(app_xtra_retention_days, value)) {
		goto out;
	}

	app_xtra_retention_days = value;
	(void)app_save_u32_setting("app/xtra_retention", app_xtra_retention_days);
	ret = bg9x_gps_xtra_policy_set(app_xtra_enabled != 0U,
				       (uint8_t)app_xtra_retention_days,
				       (uint8_t)app_xtra_mode);
	if (ret < 0) {
		status = GOLIOTH_SETTINGS_GENERAL_ERROR;
		goto out;
	}
	app_runtime_cfg_dirty = true;
	LOG_INF("settings XTRA_RETENTION=%u", app_xtra_retention_days);
out:
	k_mutex_unlock(&app_settings_cb_lock);
	return status;
}

static enum golioth_settings_status on_xtra_mode_setting(int32_t new_value, void *arg)
{
	int ret;
	uint32_t value = (uint32_t)new_value;
	enum golioth_settings_status status = GOLIOTH_SETTINGS_SUCCESS;

	ARG_UNUSED(arg);
	app_note_settings_activity();
	k_mutex_lock(&app_settings_cb_lock, K_FOREVER);
	if (app_setting_u32_duplicate(&app_cloud_xtra_mode_cache, value)) {
		goto out;
	}
	if (value > 2U) {
		status = GOLIOTH_SETTINGS_GENERAL_ERROR;
		goto out;
	}
	if (app_setting_u32_unchanged(app_xtra_mode, value)) {
		goto out;
	}

	app_xtra_mode = value;
	(void)app_save_u32_setting("app/xtra_mode", app_xtra_mode);
	ret = bg9x_gps_xtra_policy_set(app_xtra_enabled != 0U,
				       (uint8_t)app_xtra_retention_days,
				       (uint8_t)app_xtra_mode);
	if (ret < 0) {
		status = GOLIOTH_SETTINGS_GENERAL_ERROR;
		goto out;
	}
	app_runtime_cfg_dirty = true;
	LOG_INF("settings XTRA_MODE=%u", app_xtra_mode);
out:
	k_mutex_unlock(&app_settings_cb_lock);
	return status;
}

static void app_golioth_settings_register(void)
{
	if (client == NULL || app_golioth_settings_ctx != NULL) {
		return;
	}

	app_golioth_settings_ctx = golioth_settings_init(client);
	if (app_golioth_settings_ctx == NULL) {
		LOG_ERR("golioth settings init failed");
		return;
	}

	app_seed_cloud_setting_caches();

	(void)golioth_settings_register_int_with_range(app_golioth_settings_ctx,
						       "TRACKING_INTERVAL_S",
						       60, 86400,
						       on_tracking_interval_setting,
						       NULL);
	(void)golioth_settings_register_int_with_range(app_golioth_settings_ctx,
						       "PARKED_INTERVAL_S",
						       60, 86400,
						       on_parked_interval_setting,
						       NULL);
	(void)golioth_settings_register_int_with_range(app_golioth_settings_ctx,
						       "MOTION_THRESHOLD_MG",
						       100, 2000,
						       on_motion_threshold_setting,
						       NULL);
	(void)golioth_settings_register_int_with_range(app_golioth_settings_ctx,
						       "MOTION_DURATION_SAMPLES",
						       1, 50,
						       on_motion_duration_setting,
						       NULL);
	(void)golioth_settings_register_int_with_range(app_golioth_settings_ctx,
						       "TRACKING_SPEED_THRESHOLD_KMH",
						       1, 120,
						       on_tracking_speed_threshold_setting,
						       NULL);
	(void)golioth_settings_register_int_with_range(app_golioth_settings_ctx,
						       "TRACKING_STOP_COUNT",
						       1, 10,
						       on_tracking_stop_count_setting,
						       NULL);
	(void)golioth_settings_register_int_with_range(app_golioth_settings_ctx,
						       "GPS_TIMEOUT",
						       10, 1800,
						       on_gps_timeout_setting,
						       NULL);
	(void)golioth_settings_register_int_with_range(app_golioth_settings_ctx,
						       "CELLULAR_TIMEOUT",
						       10, 1800,
						       on_cellular_timeout_setting,
						       NULL);
	(void)golioth_settings_register_bool(app_golioth_settings_ctx,
					     "GPS_ENABLED",
					     on_gps_enabled_setting,
					     NULL);
	(void)golioth_settings_register_bool(app_golioth_settings_ctx,
					     "PSM_ENABLED",
					     on_psm_enabled_setting,
					     NULL);
	(void)golioth_settings_register_bool(app_golioth_settings_ctx,
					     "WAKE_ANCHORED_INTERVALS",
					     on_wake_anchored_intervals_setting,
					     NULL);
	(void)golioth_settings_register_bool(app_golioth_settings_ctx,
					     "CBOR_ENABLED",
					     on_cbor_enabled_setting,
					     NULL);
	(void)golioth_settings_register_bool(app_golioth_settings_ctx,
					     "MINIFIED_PAYLOAD",
					     on_minified_payload_setting,
					     NULL);
	(void)golioth_settings_register_bool(app_golioth_settings_ctx,
					     "XTRA_ENABLED",
					     on_xtra_enabled_setting,
					     NULL);
	(void)golioth_settings_register_int_with_range(app_golioth_settings_ctx,
						       "XTRA_RETENTION",
						       1, 7,
						       on_xtra_retention_setting,
						       NULL);
	(void)golioth_settings_register_int_with_range(app_golioth_settings_ctx,
						       "XTRA_MODE",
						       0, 2,
						       on_xtra_mode_setting,
						       NULL);

	LOG_INF("Golioth settings registered");
	app_runtime_cfg_dirty = true;
}

static void app_golioth_settings_unregister(void)
{
	if (app_golioth_settings_ctx == NULL) {
		return;
	}

	(void)golioth_settings_deinit(app_golioth_settings_ctx);
	app_golioth_settings_ctx = NULL;
}

static void publish_status_marker(char code, const char *reason)
{
#if defined(CONFIG_APP_STATUS_MARKERS)
	char value[2] = {
		code,
		'\0',
	};
	enum golioth_status status;

	if (!app_status_marker_cloud_enabled()) {
		LOG_INF("status marker local-only value=%c reason=%s", code,
			reason != NULL ? reason : "n/a");
		return;
	}

	status = publish_tracker_string_state("status", value);
	LOG_INF("status state status=%d value=%c reason=%s", status, code,
		reason != NULL ? reason : "n/a");
#else
	ARG_UNUSED(code);
	ARG_UNUSED(reason);
#endif
}

static enum golioth_status delete_lightdb_state(const char *path)
{
	enum golioth_status status;

	status = golioth_lightdb_delete_sync(client, path, APP_STATE_TIMEOUT_SECONDS);
	if (status == GOLIOTH_OK) {
		app_cycle_tx_bytes_add((uint32_t)strlen(path));
	}

	return status;
}

static enum golioth_status delete_tracker_state(const char *path)
{
	char full_path[APP_STATE_PATH_MAX];

	if (!app_path_join(full_path, sizeof(full_path), APP_TRACKER_STATE_ROOT, path)) {
		return GOLIOTH_ERR_INVALID_FORMAT;
	}

	return delete_lightdb_state(full_path);
}

static void prune_legacy_command_state_once(void)
{
	enum golioth_status status;

	if (app_legacy_command_state_pruned) {
		return;
	}

	status = delete_lightdb_state("messages");
	app_legacy_command_state_pruned = true;
	if (status == GOLIOTH_OK) {
		LOG_INF("legacy command state pruned");
	}
}

static void prune_tracker_gps_state(void)
{
	static const char *const gps_paths[] = {
		"lat_e6",
		"lon_e6",
		"alt_dm",
		"speed_x10",
		"heading_x10",
		"hdop_x10",
		"vdop_x10",
		"pdop_x10",
		"mag_var_x10",
		"sats_used",
		"sats_view",
		"fix_time_ms",
		"gps_time",
	};

	for (size_t i = 0; i < ARRAY_SIZE(gps_paths); i++) {
		(void)delete_tracker_state(gps_paths[i]);
	}
}

#if defined(CONFIG_APP_GOLIOTH_STREAM_ENABLE)
static void format_gps_time_rfc3339(const struct bg9x_gps_fix *fix, char *buf, size_t buf_size)
{
	if (buf_size == 0) {
		return;
	}

	buf[0] = '\0';
	if (fix == NULL || !fix->valid || fix->year == 0) {
		return;
	}

	snprintk(buf, buf_size, "%04u-%02u-%02uT%02u:%02u:%02u.%03uZ",
		 fix->year, fix->month, fix->day,
		 fix->utc_hour, fix->utc_min, fix->utc_sec, fix->utc_ms);
}

static void format_network_time_rfc3339(const struct bg9x_network_time *ntime, char *buf, size_t buf_size)
{
	int tz_hours;
	int tz_minutes;

	if (buf_size == 0) {
		return;
	}

	buf[0] = '\0';
	if (ntime == NULL || !ntime->valid) {
		return;
	}

	tz_hours = ntime->tz_quarter_hours / 4;
	tz_minutes = abs(ntime->tz_quarter_hours % 4) * 15;
	snprintk(buf, buf_size, "%04u-%02u-%02uT%02u:%02u:%02u%+03d:%02d",
		 ntime->year, ntime->month, ntime->day,
		 ntime->hour, ntime->min, ntime->sec,
		 tz_hours, tz_minutes);
}
#endif

static const char *tracker_field_key(enum tracker_field_id field_id, bool minified)
{
	switch (field_id) {
#define TRACKER_FIELD_CASE(id, short_key, long_key) \
	case TRACKER_FIELD_##id: \
		return minified ? short_key : long_key;
	TRACKER_FIELD_ALIAS_MAP(TRACKER_FIELD_CASE)
#undef TRACKER_FIELD_CASE
	default:
		return "";
	}
}

static bool tracker_cbor_enabled(void)
{
#if defined(CONFIG_APP_GOLIOTH_STREAM_COMPACT_CBOR)
	return app_cbor_enabled != 0U;
#else
	return false;
#endif
}

static bool tracker_minified_payload_enabled(void)
{
	return app_minified_payload != 0U;
}

static bool tracker_pipeline_state_enabled(void)
{
	return true;
}

static const char *tracker_stream_path(void)
{
#if defined(CONFIG_APP_GOLIOTH_STREAM_PATH)
	return tracker_minified_payload_enabled() ?
		CONFIG_APP_GOLIOTH_STREAM_PATH :
		CONFIG_APP_GOLIOTH_STREAM_PATH "-expanded";
#else
	return tracker_minified_payload_enabled() ? "tracker" : "tracker-expanded";
#endif
}

static bool json_appendf(char *buf, size_t buf_size, int *len, const char *fmt, ...)
{
	va_list args;
	int wrote;

	if (*len < 0 || *len >= (int)buf_size) {
		return false;
	}

	va_start(args, fmt);
	wrote = vsnprintk(buf + *len, buf_size - (size_t)*len, fmt, args);
	va_end(args);
	if (wrote < 0 || (*len + wrote) >= (int)buf_size) {
		return false;
	}

	*len += wrote;
	return true;
}

static bool json_append_sep(char *buf, size_t buf_size, int *len, bool *first)
{
	if (*first) {
		*first = false;
		return true;
	}

	return json_appendf(buf, buf_size, len, ",");
}

static bool json_append_i32_kv(char *buf, size_t buf_size, int *len, bool *first,
			       enum tracker_field_id key, int32_t value, bool minified)
{
	if (!json_append_sep(buf, buf_size, len, first)) {
		return false;
	}

	return json_appendf(buf, buf_size, len, "\"%s\":%d",
			    tracker_field_key(key, minified), value);
}

static bool json_append_u32_kv(char *buf, size_t buf_size, int *len, bool *first,
			       enum tracker_field_id key, uint32_t value, bool minified)
{
	if (!json_append_sep(buf, buf_size, len, first)) {
		return false;
	}

	return json_appendf(buf, buf_size, len, "\"%s\":%u",
			    tracker_field_key(key, minified), value);
}

static bool json_append_text_kv(char *buf, size_t buf_size, int *len, bool *first,
				enum tracker_field_id key, const char *value, bool minified)
{
	if (!json_append_sep(buf, buf_size, len, first)) {
		return false;
	}

	return json_appendf(buf, buf_size, len, "\"%s\":\"%s\"",
			    tracker_field_key(key, minified), value);
}

#if defined(CONFIG_APP_GOLIOTH_STREAM_ENABLE) && defined(CONFIG_APP_GOLIOTH_STREAM_COMPACT_CBOR)
static bool cbor_put_text_kv(zcbor_state_t *zs, enum tracker_field_id key, const char *value,
			     bool minified)
{
	const char *field_key = tracker_field_key(key, minified);

	return zcbor_tstr_encode_ptr(zs, field_key, strlen(field_key)) &&
	       zcbor_tstr_encode_ptr(zs, value, strlen(value));
}

static bool cbor_put_i32_kv(zcbor_state_t *zs, enum tracker_field_id key, int32_t value,
			    bool minified)
{
	const char *field_key = tracker_field_key(key, minified);

	return zcbor_tstr_encode_ptr(zs, field_key, strlen(field_key)) &&
	       zcbor_int32_put(zs, value);
}

static bool cbor_put_u32_kv(zcbor_state_t *zs, enum tracker_field_id key, uint32_t value,
			    bool minified)
{
	const char *field_key = tracker_field_key(key, minified);

	return zcbor_tstr_encode_ptr(zs, field_key, strlen(field_key)) &&
	       zcbor_uint32_put(zs, value);
}

static bool encode_tracker_stream_cbor(uint8_t *payload, size_t payload_size,
				       size_t *encoded_size, int counter,
				       bool heartbeat_only, int batt_mv,
				       uint32_t next_wake_seconds,
				       const struct bg9x_gps_fix *fix,
				       const struct bg9x_cell_info *cell,
				       const struct bg9x_network_time *ntime,
				       bool minified)
{
	ZCBOR_STATE_E(zse, 2, payload, payload_size, 0);
	char gps_time[40];
	char net_time[40];
	char event_time[40];
	char mode[2];
	bool ok;

	format_gps_time_rfc3339(fix, gps_time, sizeof(gps_time));
	format_network_time_rfc3339(ntime, net_time, sizeof(net_time));
	if (gps_time[0] != '\0') {
		strcpy(event_time, gps_time);
	} else if (net_time[0] != '\0') {
		strcpy(event_time, net_time);
	} else {
		event_time[0] = '\0';
	}

	mode[0] = app_tracker_mode_code(app_tracker_mode);
	mode[1] = '\0';

	ok = zcbor_map_start_encode(zse, 0);
	ok = ok && cbor_put_i32_kv(zse, TRACKER_FIELD_CYCLE, counter, minified);
	ok = ok && cbor_put_u32_kv(zse, TRACKER_FIELD_BOOT_COUNT, app_boot_count, minified);
	ok = ok && cbor_put_u32_kv(zse, TRACKER_FIELD_TOTAL_CYCLES, app_total_cycles, minified);
	ok = ok && cbor_put_i32_kv(zse, TRACKER_FIELD_UPTIME_MS, (int32_t)k_uptime_get(), minified);
	ok = ok && cbor_put_i32_kv(zse, TRACKER_FIELD_HEARTBEAT, heartbeat_only ? 1 : 0,
				   minified);
	ok = ok && cbor_put_text_kv(zse, TRACKER_FIELD_MODE, mode, minified);
	ok = ok && cbor_put_u32_kv(zse, TRACKER_FIELD_NEXT_WAKE_S, next_wake_seconds, minified);
	ok = ok && cbor_put_u32_kv(zse, TRACKER_FIELD_MOTION_SEEN,
				   app_tracking_motion_events_last_interval > 0U ? 1U : 0U,
				   minified);
	ok = ok && cbor_put_i32_kv(zse, TRACKER_FIELD_TEMP_C_X10, last_temp_c10, minified);
	ok = ok && cbor_put_i32_kv(zse, TRACKER_FIELD_HUMIDITY_X10, last_humid_c10, minified);
	ok = ok && cbor_put_i32_kv(zse, TRACKER_FIELD_ACCEL_X, last_accel_x, minified);
	ok = ok && cbor_put_i32_kv(zse, TRACKER_FIELD_ACCEL_Y, last_accel_y, minified);
	ok = ok && cbor_put_i32_kv(zse, TRACKER_FIELD_ACCEL_Z, last_accel_z, minified);
	if (last_batt_pct >= 0) {
		ok = ok && cbor_put_i32_kv(zse, TRACKER_FIELD_BAT_PCT, last_batt_pct, minified);
	}
	ok = ok && cbor_put_i32_kv(zse, TRACKER_FIELD_BAT_MV, batt_mv >= 0 ? batt_mv : 0,
				   minified);

	if (event_time[0] != '\0') {
		ok = ok && cbor_put_text_kv(zse, TRACKER_FIELD_EVENT_TIME, event_time, minified);
	}
	if (gps_time[0] != '\0') {
		ok = ok && cbor_put_text_kv(zse, TRACKER_FIELD_GPS_TIME, gps_time, minified);
	}
	if (net_time[0] != '\0') {
		ok = ok && cbor_put_text_kv(zse, TRACKER_FIELD_NET_TIME, net_time, minified);
	}
	if (ntime != NULL && ntime->valid) {
		ok = ok && cbor_put_i32_kv(zse, TRACKER_FIELD_TZ, ntime->tz_quarter_hours / 4,
					   minified);
	}

	if (fix != NULL && fix->valid) {
		ok = ok && cbor_put_i32_kv(zse, TRACKER_FIELD_LAT_E6,
					   (int32_t)(fix->latitude * 1000000.0), minified);
		ok = ok && cbor_put_i32_kv(zse, TRACKER_FIELD_LON_E6,
					   (int32_t)(fix->longitude * 1000000.0), minified);
		ok = ok && cbor_put_i32_kv(zse, TRACKER_FIELD_ALT_DM,
					   (int32_t)(fix->altitude * 10.0f), minified);
		ok = ok && cbor_put_i32_kv(zse, TRACKER_FIELD_SPEED_X10,
					   (int32_t)(fix->speed_kmh * 10.0f), minified);
		ok = ok && cbor_put_i32_kv(zse, TRACKER_FIELD_HEADING_X10,
					   (int32_t)(fix->course * 10.0f), minified);
		ok = ok && cbor_put_i32_kv(zse, TRACKER_FIELD_FIX_TYPE, fix->fix_type, minified);
		ok = ok && cbor_put_u32_kv(zse, TRACKER_FIELD_FIX_TIME_MS, fix->fix_time_ms,
					   minified);
		ok = ok && cbor_put_i32_kv(zse, TRACKER_FIELD_SATS_USED, fix->sats_in_use,
					   minified);
		ok = ok && cbor_put_i32_kv(zse, TRACKER_FIELD_SATS_VIEW, fix->sats_in_view,
					   minified);
		ok = ok && cbor_put_i32_kv(zse, TRACKER_FIELD_HDOP_X10,
					   (int32_t)(fix->hdop * 10.0f), minified);
		ok = ok && cbor_put_i32_kv(zse, TRACKER_FIELD_PDOP_X10,
					   (int32_t)(fix->pdop * 10.0f), minified);
		ok = ok && cbor_put_i32_kv(zse, TRACKER_FIELD_VDOP_X10,
					   (int32_t)(fix->vdop * 10.0f), minified);
		ok = ok && cbor_put_i32_kv(zse, TRACKER_FIELD_MAG_VAR_X10,
					   (int32_t)(fix->magnetic_variation * 10.0f),
					   minified);
	}

	if (cell != NULL && cell->valid) {
		ok = ok && cbor_put_i32_kv(zse, TRACKER_FIELD_RSRP, cell->rsrp, minified);
		ok = ok && cbor_put_i32_kv(zse, TRACKER_FIELD_RSRQ, cell->rsrq, minified);
		ok = ok && cbor_put_i32_kv(zse, TRACKER_FIELD_SINR, cell->sinr, minified);
		ok = ok && cbor_put_i32_kv(zse, TRACKER_FIELD_CELL_ID, (int32_t)cell->cell_id,
					   minified);
		ok = ok && cbor_put_i32_kv(zse, TRACKER_FIELD_PCI, cell->pci, minified);
		ok = ok && cbor_put_i32_kv(zse, TRACKER_FIELD_EARFCN, (int32_t)cell->earfcn,
					   minified);
		ok = ok && cbor_put_i32_kv(zse, TRACKER_FIELD_MCC, cell->mcc, minified);
		ok = ok && cbor_put_i32_kv(zse, TRACKER_FIELD_MNC, cell->mnc, minified);
		ok = ok && cbor_put_i32_kv(zse, TRACKER_FIELD_BAND, cell->band, minified);
	}

	{
		struct bg9x_runtime_stats stats = {0};

		if (bg9x_runtime_stats_get(&stats) == 0) {
			ok = ok && cbor_put_u32_kv(zse, TRACKER_FIELD_BG95_NATIVE_WAKE_COUNT,
						   stats.native_wake_count, minified);
			ok = ok && cbor_put_u32_kv(zse, TRACKER_FIELD_BG95_EMERGENCY_REINIT_COUNT,
						   stats.emergency_reinit_count, minified);
		}
	}

	ok = ok && zcbor_map_end_encode(zse, 0);
	if (!ok) {
		return false;
	}

	*encoded_size = (size_t)((uintptr_t)zse->payload - (uintptr_t)payload);
	return true;
}
#endif

enum tracker_publish_result {
	TRACKER_PUBLISH_FAILED = 0,
	TRACKER_PUBLISH_SENT,
	TRACKER_PUBLISH_TIMEOUT,
};

static enum tracker_publish_result publish_tracker_stream_record(
	int counter, bool heartbeat_only, int batt_mv, uint32_t next_wake_seconds,
	const struct bg9x_gps_fix *fix, const struct bg9x_cell_info *cell,
	const struct bg9x_network_time *ntime)
{
#if defined(CONFIG_APP_GOLIOTH_STREAM_ENABLE)
	uint8_t cbor_payload[320];
	char json_payload[512];
	size_t payload_size = 0;
	int len = 0;
	enum golioth_status status;
	bool minified = tracker_minified_payload_enabled();
	const char *stream_path = tracker_stream_path();
	char gps_time[40];
	char net_time[40];
	char event_time[40];
	char mode[2];
	bool first = true;

#if defined(CONFIG_APP_GOLIOTH_STREAM_TRACKING_ONLY)
	if (heartbeat_only) {
		return TRACKER_PUBLISH_SENT;
	}
#endif

	mode[0] = app_tracker_mode_code(app_tracker_mode);
	mode[1] = '\0';
	format_gps_time_rfc3339(fix, gps_time, sizeof(gps_time));
	format_network_time_rfc3339(ntime, net_time, sizeof(net_time));
	if (gps_time[0] != '\0') {
		strcpy(event_time, gps_time);
	} else if (net_time[0] != '\0') {
		strcpy(event_time, net_time);
	} else {
		event_time[0] = '\0';
	}

	if (tracker_cbor_enabled()) {
#if defined(CONFIG_APP_GOLIOTH_STREAM_COMPACT_CBOR)
		if (!encode_tracker_stream_cbor(cbor_payload, sizeof(cbor_payload), &payload_size,
						counter, heartbeat_only, batt_mv,
						next_wake_seconds, fix, cell, ntime, minified)) {
			LOG_WRN("tracker stream CBOR encode failed");
			return TRACKER_PUBLISH_FAILED;
		}

		status = golioth_stream_set_sync(client, stream_path,
						 GOLIOTH_CONTENT_TYPE_CBOR,
						 cbor_payload, payload_size,
						 TRACKER_STREAM_TIMEOUT_SECONDS);
		if (status != GOLIOTH_OK) {
			if (status == GOLIOTH_ERR_TIMEOUT) {
				LOG_WRN("tracker stream cbor timeout status=%d len=%u path=%s min=%u; suppressing retry to avoid duplicate uploads",
					status, (unsigned int)payload_size, stream_path,
					minified ? 1U : 0U);
			} else {
				LOG_WRN("tracker stream cbor retrying status=%d len=%u path=%s min=%u",
					status, (unsigned int)payload_size, stream_path,
					minified ? 1U : 0U);
				k_sleep(K_MSEC(500));
				app_wdt_feed();
				status = golioth_stream_set_sync(client, stream_path,
							 GOLIOTH_CONTENT_TYPE_CBOR,
							 cbor_payload, payload_size,
							 TRACKER_STREAM_TIMEOUT_SECONDS);
			}
		}
#if defined(CONFIG_APP_LOG_RAW_TRACKER_PAYLOAD)
		{
			char hex_payload[(sizeof(cbor_payload) * 2U) + 1U];

			(void)bin2hex(cbor_payload, payload_size, hex_payload, sizeof(hex_payload));
			app_record_tracker_payload(stream_path, payload_size, hex_payload);
			app_log_tracker_payload_debug("cbor", minified);
		}
#else
		app_record_tracker_payload(stream_path, payload_size, "");
		app_log_tracker_payload_debug("cbor", minified);
#endif
		LOG_INF("tracker stream cbor status=%d len=%u path=%s min=%u",
			status, (unsigned int)payload_size, stream_path, minified ? 1U : 0U);
		if (status == GOLIOTH_OK || status == GOLIOTH_ERR_TIMEOUT) {
			app_cycle_tracker_tx_bytes_add((uint32_t)(strlen(stream_path) + payload_size));
		}
		if (status == GOLIOTH_OK) {
			return TRACKER_PUBLISH_SENT;
		}
		if (status == GOLIOTH_ERR_TIMEOUT) {
			return TRACKER_PUBLISH_TIMEOUT;
		}
		return TRACKER_PUBLISH_FAILED;
#else
		ARG_UNUSED(cbor_payload);
		ARG_UNUSED(payload_size);
		LOG_WRN("tracker CBOR disabled at build time");
		return TRACKER_PUBLISH_FAILED;
#endif
	}

	if (!json_appendf(json_payload, sizeof(json_payload), &len, "{")) {
		LOG_WRN("tracker stream JSON start failed");
		return TRACKER_PUBLISH_FAILED;
	}
	if (!json_append_i32_kv(json_payload, sizeof(json_payload), &len, &first,
				TRACKER_FIELD_CYCLE, counter, minified) ||
	    !json_append_u32_kv(json_payload, sizeof(json_payload), &len, &first,
				 TRACKER_FIELD_BOOT_COUNT, app_boot_count, minified) ||
	    !json_append_u32_kv(json_payload, sizeof(json_payload), &len, &first,
				 TRACKER_FIELD_TOTAL_CYCLES, app_total_cycles, minified) ||
	    !json_append_i32_kv(json_payload, sizeof(json_payload), &len, &first,
				TRACKER_FIELD_UPTIME_MS, (int32_t)k_uptime_get(), minified) ||
	    !json_append_i32_kv(json_payload, sizeof(json_payload), &len, &first,
				TRACKER_FIELD_HEARTBEAT, heartbeat_only ? 1 : 0, minified) ||
	    !json_append_text_kv(json_payload, sizeof(json_payload), &len, &first,
				 TRACKER_FIELD_MODE, mode, minified) ||
	    !json_append_u32_kv(json_payload, sizeof(json_payload), &len, &first,
				 TRACKER_FIELD_NEXT_WAKE_S, next_wake_seconds, minified) ||
	    !json_append_u32_kv(json_payload, sizeof(json_payload), &len, &first,
				 TRACKER_FIELD_MOTION_SEEN,
				 app_tracking_motion_events_last_interval > 0U ? 1U : 0U,
				 minified) ||
	    !json_append_i32_kv(json_payload, sizeof(json_payload), &len, &first,
				TRACKER_FIELD_TEMP_C_X10, last_temp_c10, minified) ||
	    !json_append_i32_kv(json_payload, sizeof(json_payload), &len, &first,
				TRACKER_FIELD_HUMIDITY_X10, last_humid_c10, minified) ||
	    !json_append_i32_kv(json_payload, sizeof(json_payload), &len, &first,
				TRACKER_FIELD_ACCEL_X, last_accel_x, minified) ||
	    !json_append_i32_kv(json_payload, sizeof(json_payload), &len, &first,
				TRACKER_FIELD_ACCEL_Y, last_accel_y, minified) ||
	    !json_append_i32_kv(json_payload, sizeof(json_payload), &len, &first,
				TRACKER_FIELD_ACCEL_Z, last_accel_z, minified) ||
	    (last_batt_pct >= 0 &&
	     !json_append_i32_kv(json_payload, sizeof(json_payload), &len, &first,
				 TRACKER_FIELD_BAT_PCT, last_batt_pct, minified)) ||
	    !json_append_i32_kv(json_payload, sizeof(json_payload), &len, &first,
				 TRACKER_FIELD_BAT_MV, batt_mv >= 0 ? batt_mv : 0, minified)) {
		LOG_WRN("tracker stream JSON base encode failed");
		return TRACKER_PUBLISH_FAILED;
	}

	if (event_time[0] != '\0' &&
	    !json_append_text_kv(json_payload, sizeof(json_payload), &len, &first,
				 TRACKER_FIELD_EVENT_TIME, event_time, minified)) {
		LOG_WRN("tracker stream JSON event_time append failed");
		return TRACKER_PUBLISH_FAILED;
	}
	if (gps_time[0] != '\0' &&
	    !json_append_text_kv(json_payload, sizeof(json_payload), &len, &first,
				 TRACKER_FIELD_GPS_TIME, gps_time, minified)) {
		LOG_WRN("tracker stream JSON gps_time append failed");
		return TRACKER_PUBLISH_FAILED;
	}
	if (net_time[0] != '\0' &&
	    !json_append_text_kv(json_payload, sizeof(json_payload), &len, &first,
				 TRACKER_FIELD_NET_TIME, net_time, minified)) {
		LOG_WRN("tracker stream JSON net_time append failed");
		return TRACKER_PUBLISH_FAILED;
	}
	if (ntime != NULL && ntime->valid &&
	    !json_append_i32_kv(json_payload, sizeof(json_payload), &len, &first,
				TRACKER_FIELD_TZ, ntime->tz_quarter_hours / 4, minified)) {
		LOG_WRN("tracker stream JSON tz append failed");
		return TRACKER_PUBLISH_FAILED;
	}

	if (fix != NULL && fix->valid) {
		if (!json_append_i32_kv(json_payload, sizeof(json_payload), &len, &first,
					TRACKER_FIELD_LAT_E6,
					(int32_t)(fix->latitude * 1000000.0), minified) ||
		    !json_append_i32_kv(json_payload, sizeof(json_payload), &len, &first,
					TRACKER_FIELD_LON_E6,
					(int32_t)(fix->longitude * 1000000.0), minified) ||
		    !json_append_i32_kv(json_payload, sizeof(json_payload), &len, &first,
					TRACKER_FIELD_ALT_DM,
					(int32_t)(fix->altitude * 10.0f), minified) ||
		    !json_append_i32_kv(json_payload, sizeof(json_payload), &len, &first,
					TRACKER_FIELD_SPEED_X10,
					(int32_t)(fix->speed_kmh * 10.0f), minified) ||
		    !json_append_i32_kv(json_payload, sizeof(json_payload), &len, &first,
					TRACKER_FIELD_HEADING_X10,
					(int32_t)(fix->course * 10.0f), minified) ||
		    !json_append_i32_kv(json_payload, sizeof(json_payload), &len, &first,
					TRACKER_FIELD_FIX_TYPE, fix->fix_type, minified) ||
		    !json_append_u32_kv(json_payload, sizeof(json_payload), &len, &first,
					TRACKER_FIELD_FIX_TIME_MS, fix->fix_time_ms, minified) ||
		    !json_append_i32_kv(json_payload, sizeof(json_payload), &len, &first,
					TRACKER_FIELD_SATS_USED, fix->sats_in_use, minified) ||
		    !json_append_i32_kv(json_payload, sizeof(json_payload), &len, &first,
					TRACKER_FIELD_SATS_VIEW, fix->sats_in_view, minified) ||
		    !json_append_i32_kv(json_payload, sizeof(json_payload), &len, &first,
					TRACKER_FIELD_HDOP_X10,
					(int32_t)(fix->hdop * 10.0f), minified) ||
		    !json_append_i32_kv(json_payload, sizeof(json_payload), &len, &first,
					TRACKER_FIELD_PDOP_X10,
					(int32_t)(fix->pdop * 10.0f), minified) ||
		    !json_append_i32_kv(json_payload, sizeof(json_payload), &len, &first,
					TRACKER_FIELD_VDOP_X10,
					(int32_t)(fix->vdop * 10.0f), minified) ||
		    !json_append_i32_kv(json_payload, sizeof(json_payload), &len, &first,
					TRACKER_FIELD_MAG_VAR_X10,
					(int32_t)(fix->magnetic_variation * 10.0f), minified)) {
			LOG_WRN("tracker stream JSON GPS append failed");
			return TRACKER_PUBLISH_FAILED;
		}
	}

	if (cell != NULL && cell->valid) {
		if (!json_append_i32_kv(json_payload, sizeof(json_payload), &len, &first,
					TRACKER_FIELD_RSRP, cell->rsrp, minified) ||
		    !json_append_i32_kv(json_payload, sizeof(json_payload), &len, &first,
					TRACKER_FIELD_RSRQ, cell->rsrq, minified) ||
		    !json_append_i32_kv(json_payload, sizeof(json_payload), &len, &first,
					TRACKER_FIELD_SINR, cell->sinr, minified) ||
		    !json_append_i32_kv(json_payload, sizeof(json_payload), &len, &first,
					TRACKER_FIELD_CELL_ID, (int32_t)cell->cell_id, minified) ||
		    !json_append_i32_kv(json_payload, sizeof(json_payload), &len, &first,
					TRACKER_FIELD_PCI, cell->pci, minified) ||
		    !json_append_i32_kv(json_payload, sizeof(json_payload), &len, &first,
					TRACKER_FIELD_EARFCN, (int32_t)cell->earfcn, minified) ||
		    !json_append_i32_kv(json_payload, sizeof(json_payload), &len, &first,
					TRACKER_FIELD_MCC, cell->mcc, minified) ||
		    !json_append_i32_kv(json_payload, sizeof(json_payload), &len, &first,
					TRACKER_FIELD_MNC, cell->mnc, minified) ||
		    !json_append_i32_kv(json_payload, sizeof(json_payload), &len, &first,
					TRACKER_FIELD_BAND, cell->band, minified)) {
			LOG_WRN("tracker stream JSON cell append failed");
			return TRACKER_PUBLISH_FAILED;
		}
	}

	{
		struct bg9x_runtime_stats stats = {0};

		if (bg9x_runtime_stats_get(&stats) == 0) {
			if (!json_append_u32_kv(json_payload, sizeof(json_payload), &len, &first,
						TRACKER_FIELD_BG95_NATIVE_WAKE_COUNT,
						stats.native_wake_count, minified) ||
			    !json_append_u32_kv(json_payload, sizeof(json_payload), &len, &first,
						TRACKER_FIELD_BG95_EMERGENCY_REINIT_COUNT,
						stats.emergency_reinit_count, minified)) {
				LOG_WRN("tracker stream JSON wake stats append failed");
				return TRACKER_PUBLISH_FAILED;
			}
		}
	}

	if (!json_appendf(json_payload, sizeof(json_payload), &len, "}")) {
		LOG_WRN("tracker stream JSON finalization failed");
		return TRACKER_PUBLISH_FAILED;
	}

#if defined(CONFIG_APP_LOG_RAW_TRACKER_PAYLOAD)
	app_record_tracker_payload(stream_path, (size_t)len, json_payload);
	app_log_tracker_payload_debug("json", minified);
#else
	app_record_tracker_payload(stream_path, (size_t)len, "");
	app_log_tracker_payload_debug("json", minified);
#endif
	status = golioth_stream_set_sync(client, stream_path,
					 GOLIOTH_CONTENT_TYPE_JSON,
					 (const uint8_t *)json_payload, (size_t)len,
					 TRACKER_STREAM_TIMEOUT_SECONDS);
	if (status != GOLIOTH_OK) {
		if (status == GOLIOTH_ERR_TIMEOUT) {
			LOG_WRN("tracker stream json timeout status=%d len=%d path=%s min=%u; suppressing retry to avoid duplicate uploads",
				status, len, stream_path, minified ? 1U : 0U);
		} else {
			LOG_WRN("tracker stream json retrying status=%d len=%d path=%s min=%u",
				status, len, stream_path, minified ? 1U : 0U);
			k_sleep(K_MSEC(500));
			app_wdt_feed();
			status = golioth_stream_set_sync(client, stream_path,
						 GOLIOTH_CONTENT_TYPE_JSON,
						 (const uint8_t *)json_payload, (size_t)len,
						 TRACKER_STREAM_TIMEOUT_SECONDS);
		}
	}
	LOG_INF("tracker stream json status=%d len=%d path=%s min=%u",
		status, len, stream_path, minified ? 1U : 0U);
	if (status == GOLIOTH_OK || status == GOLIOTH_ERR_TIMEOUT) {
		app_cycle_tracker_tx_bytes_add((uint32_t)(strlen(stream_path) + len));
	}
	if (status == GOLIOTH_OK) {
		return TRACKER_PUBLISH_SENT;
	}
	if (status == GOLIOTH_ERR_TIMEOUT) {
		return TRACKER_PUBLISH_TIMEOUT;
	}
	return TRACKER_PUBLISH_FAILED;
#else
	ARG_UNUSED(counter);
	ARG_UNUSED(heartbeat_only);
	ARG_UNUSED(batt_mv);
	ARG_UNUSED(fix);
	ARG_UNUSED(cell);
	ARG_UNUSED(ntime);
	return TRACKER_PUBLISH_FAILED;
#endif
}

static uint32_t format_cycle_log_payload(char *msg, size_t msg_size, int counter,
					 bool heartbeat_only, int batt_mv,
					 uint32_t next_wake_seconds)
{
	uint32_t total_tx_bytes = app_cycle_tx_bytes;

	while (true) {
		uint32_t projected_bytes_today = app_bytes_today + total_tx_bytes;
		int msg_len;
		uint32_t next_total_tx_bytes;

		msg_len = snprintk(msg, msg_size,
				   "up=%lld cy=%d bt=%u tot=%u md=%c hb=%d bat=%d nxt=%us ms=%u byt=%u tx=%u",
				   k_uptime_get(), counter, app_boot_count, app_total_cycles,
				   app_tracker_mode_code(app_tracker_mode), heartbeat_only ? 1 : 0,
				   batt_mv >= 0 ? batt_mv : 0, next_wake_seconds,
				   app_tracking_motion_events_last_interval > 0U ? 1U : 0U,
				   projected_bytes_today, total_tx_bytes);

		if (msg_len < 0) {
			return 0U;
		}

		next_total_tx_bytes = app_cycle_tx_bytes + (uint32_t)msg_len;
		if (next_total_tx_bytes == total_tx_bytes) {
			return next_total_tx_bytes;
		}

		total_tx_bytes = next_total_tx_bytes;
	}
}

static bool send_cycle_log(int counter, bool heartbeat_only, int batt_mv,
			   uint32_t next_wake_seconds, uint32_t *tx_bytes_out)
{
	char msg[160];
	enum golioth_status status = GOLIOTH_OK;
	uint32_t tx_bytes;

	tx_bytes = format_cycle_log_payload(msg, sizeof(msg), counter, heartbeat_only,
					    batt_mv, next_wake_seconds);
	app_last_cycle_log_len = (uint32_t)strlen(msg);

	if (app_cloud_cycle_log_enabled) {
		status = golioth_log_info_sync(client, APP_GOLIOTH_LOG_MODULE, msg,
					 APP_LOG_TIMEOUT_SECONDS);
		LOG_INF("cycle log status=%d %s", status, msg);
	} else {
		LOG_INF("cycle log local-only %s", msg);
	}
	LOG_INF("cycle tx breakdown tracker=%u aux=%u cycle_log=%u total=%u",
		app_cycle_tracker_tx_bytes, app_cycle_aux_tx_bytes,
		app_last_cycle_log_len, tx_bytes);
	if (tx_bytes_out != NULL) {
		*tx_bytes_out = tx_bytes;
	}

	app_cycle_tx_bytes = tx_bytes;
	if (app_cloud_cycle_log_enabled) {
		if (status == GOLIOTH_OK || status == GOLIOTH_ERR_TIMEOUT) {
			app_bytes_today += tx_bytes;
			app_bytes_today_commit();
		}

		return status == GOLIOTH_OK;
	}

	app_bytes_today += app_cycle_tracker_tx_bytes + app_cycle_aux_tx_bytes;
	app_bytes_today_commit();
	return true;
}

static void app_bytes_today_check_rollover(void)
{
	uint32_t utc_day;
	uint32_t previous_day;
	uint32_t previous_bytes;
	int err;

	err = bg9x_utc_day_get(&utc_day);
	if (err < 0) {
		LOG_WRN("bg9x_utc_day_get failed: %d", err);
		return;
	}

	if (!app_day_boundary_valid) {
		app_day_boundary = utc_day;
		app_day_boundary_valid = true;
		app_bytes_today_commit();
		LOG_INF("Initialized bytes_today day boundary to %u", utc_day);
		return;
	}

	if (utc_day == app_day_boundary) {
		return;
	}

	previous_day = app_day_boundary;
	previous_bytes = app_bytes_today;
	app_day_boundary = utc_day;
	app_bytes_today = 0U;
	app_bytes_today_commit();
	LOG_INF("UTC day rollover prev_day=%u bytes=%u new_day=%u",
		previous_day, previous_bytes, utc_day);

	if (client != NULL) {
		char msg[96];
		enum golioth_status status;

		snprintk(msg, sizeof(msg),
			 "daily bytes rollover prev_day=%u bytes=%u new_day=%u",
			 previous_day, previous_bytes, utc_day);
		status = golioth_log_info_sync(client, APP_GOLIOTH_LOG_MODULE, msg,
					       APP_LOG_TIMEOUT_SECONDS);
		LOG_INF("daily_bytes_log status=%d msg=\"%s\"", status, msg);
		if (status == GOLIOTH_OK) {
			app_cycle_tx_bytes_add((uint32_t)strlen(msg));
		}
	}
}

static void publish_battery_state_mv(int batt_mv)
{
	enum golioth_status status;

	status = publish_tracker_int_state("bat_mv", batt_mv);
	LOG_DBG("bat_mv state status=%d value=%d", status, batt_mv);
}

static void publish_battery_state_pct(int batt_pct)
{
	enum golioth_status status;

	status = publish_tracker_int_state("bat_pct", batt_pct);
	LOG_DBG("bat_pct state status=%d value=%d", status, batt_pct);
}

static void publish_tracker_mode_state(void)
{
	enum golioth_status status;

	status = publish_tracker_int_state("mode", (int32_t)app_tracker_mode);
	LOG_DBG("mode state status=%d value=%s", status,
		app_tracker_mode_name(app_tracker_mode));
}

static void publish_motion_interrupts_state(void)
{
	enum golioth_status status;

	status = publish_tracker_int_state("motion_seen",
				      app_tracking_motion_events_last_interval > 0U ? 1 : 0);
	LOG_DBG("motion_seen state status=%d value=%u", status,
		app_tracking_motion_events_last_interval > 0U ? 1U : 0U);
}

static void publish_modem_wake_stats(void)
{
	struct bg9x_runtime_stats stats;
	enum golioth_status native_status;
	enum golioth_status emergency_status;
	int err;

	err = bg9x_runtime_stats_get(&stats);
	if (err < 0) {
		LOG_WRN("bg9x_runtime_stats_get failed: %d", err);
		return;
	}

	native_status = publish_tracker_int_state("bg95_native_wake_count",
					       (int32_t)stats.native_wake_count);
	emergency_status = publish_tracker_int_state("bg95_emergency_reinit_count",
						  (int32_t)stats.emergency_reinit_count);
	LOG_DBG("bg95 wake stats native=%u emergency=%u status=(%d,%d)",
		stats.native_wake_count, stats.emergency_reinit_count,
		native_status, emergency_status);
}

static void app_gps_fix_store(const struct bg9x_gps_fix *fix)
{
	app_gps_lat_e6 = (int32_t)(fix->latitude * 1000000.0);
	app_gps_lon_e6 = (int32_t)(fix->longitude * 1000000.0);
	app_gps_alt_cm = (int32_t)(fix->altitude * 100.0f);
	app_gps_speed_x10 = (uint32_t)(fix->speed_kmh * 10.0f);
	app_gps_course_x10 = (uint32_t)(fix->course * 10.0f);
	app_gps_sats = fix->sats_in_use;
	app_gps_fixtime_ms = fix->fix_time_ms;
	app_gps_uptime_ms = (uint32_t)k_uptime_get();

	settings_save_one("app/gps_lat", &app_gps_lat_e6, sizeof(app_gps_lat_e6));
	settings_save_one("app/gps_lon", &app_gps_lon_e6, sizeof(app_gps_lon_e6));
	settings_save_one("app/gps_alt", &app_gps_alt_cm, sizeof(app_gps_alt_cm));
	settings_save_one("app/gps_speed", &app_gps_speed_x10, sizeof(app_gps_speed_x10));
	settings_save_one("app/gps_course", &app_gps_course_x10, sizeof(app_gps_course_x10));
	settings_save_one("app/gps_sats", &app_gps_sats, sizeof(app_gps_sats));
	settings_save_one("app/gps_fixtime", &app_gps_fixtime_ms, sizeof(app_gps_fixtime_ms));
	settings_save_one("app/gps_uptime", &app_gps_uptime_ms, sizeof(app_gps_uptime_ms));

	LOG_INF("GPS fix stored: lat=%d.%06d lon=%d.%06d alt=%dcm sats=%u fix=%ums",
		app_gps_lat_e6 / 1000000, abs(app_gps_lat_e6) % 1000000,
		app_gps_lon_e6 / 1000000, abs(app_gps_lon_e6) % 1000000,
		app_gps_alt_cm, app_gps_sats, app_gps_fixtime_ms);
}

static void publish_gps_state(const struct bg9x_gps_fix *fix)
{
	if (!fix->valid) {
		prune_tracker_gps_state();
		(void)publish_tracker_int_state("fix_type", 0);
		return;
	}

	/* lat/lon as int * 1e6 for precision without float */
	int32_t lat_e6 = (int32_t)(fix->latitude * 1000000.0);
	int32_t lon_e6 = (int32_t)(fix->longitude * 1000000.0);

	(void)publish_tracker_int_state("lat_e6", lat_e6);
	(void)publish_tracker_int_state("lon_e6", lon_e6);
	(void)publish_tracker_int_state("alt_dm", (int32_t)(fix->altitude * 10.0f));
	(void)publish_tracker_int_state("fix_type", fix->fix_type);
	(void)publish_tracker_int_state("speed_x10", (int32_t)(fix->speed_kmh * 10.0f));
	(void)publish_tracker_int_state("heading_x10", (int32_t)(fix->course * 10.0f));
	(void)publish_tracker_int_state("hdop_x10", (int32_t)(fix->hdop * 10.0f));
	(void)publish_tracker_int_state("vdop_x10", (int32_t)(fix->vdop * 10.0f));
	(void)publish_tracker_int_state("pdop_x10", (int32_t)(fix->pdop * 10.0f));
	(void)publish_tracker_int_state("mag_var_x10",
				      (int32_t)(fix->magnetic_variation * 10.0f));
	(void)publish_tracker_int_state("sats_used", fix->sats_in_use);
	(void)publish_tracker_int_state("sats_view", fix->sats_in_view);
	(void)publish_tracker_int_state("fix_time_ms", (int32_t)fix->fix_time_ms);

	if (fix->year > 0) {
		char gps_time[40];

		snprintk(gps_time, sizeof(gps_time), "%04u-%02u-%02uT%02u:%02u:%02u.%03uZ",
			 fix->year, fix->month, fix->day,
			 fix->utc_hour, fix->utc_min, fix->utc_sec, fix->utc_ms);
		(void)publish_tracker_string_state("gps_time", gps_time);
	}

	LOG_DBG("GPS published: lat=%.6f lon=%.6f alt=%.1f sats=%u fix=%ums",
		fix->latitude, fix->longitude, (double)fix->altitude,
		fix->sats_in_use, fix->fix_time_ms);
}

static void publish_environment_state(void)
{
	(void)publish_tracker_int_state("accel_x", last_accel_x);
	(void)publish_tracker_int_state("accel_y", last_accel_y);
	(void)publish_tracker_int_state("accel_z", last_accel_z);
	(void)publish_tracker_int_state("temp_c_x10", last_temp_c10);
	(void)publish_tracker_int_state("humidity_x10", last_humid_c10);

	LOG_DBG("Environment published: temp=%d.%dC humid=%d.%d%%",
		last_temp_c10 / 10, abs(last_temp_c10) % 10,
		last_humid_c10 / 10, abs(last_humid_c10) % 10);
}

static void publish_cell_state(const struct bg9x_cell_info *cell)
{
	if (!cell->valid) {
		return;
	}

	(void)publish_tracker_int_state("rsrp", cell->rsrp);
	(void)publish_tracker_int_state("rsrq", cell->rsrq);
	(void)publish_tracker_int_state("sinr", cell->sinr);
	(void)publish_tracker_int_state("cell_id", (int32_t)cell->cell_id);
	(void)publish_tracker_int_state("band", cell->band);
	(void)publish_tracker_int_state("mcc", cell->mcc);
	(void)publish_tracker_int_state("mnc", cell->mnc);
	(void)publish_tracker_int_state("pci", cell->pci);
	(void)publish_tracker_int_state("earfcn", (int32_t)cell->earfcn);

	LOG_DBG("Cell published: MCC=%u MNC=%u cellID=%x RSRP=%d SINR=%d",
		cell->mcc, cell->mnc, cell->cell_id, cell->rsrp, cell->sinr);
}

static void publish_network_time_state(const struct bg9x_network_time *ntime)
{
	char net_time[40];
	int tz_hours;
	int tz_minutes;
	enum golioth_status delete_status;

	if (!ntime->valid) {
		return;
	}

	tz_hours = ntime->tz_quarter_hours / 4;
	tz_minutes = abs(ntime->tz_quarter_hours % 4) * 15;

	snprintk(net_time, sizeof(net_time), "%04u-%02u-%02uT%02u:%02u:%02u%+03d:%02d",
		 ntime->year, ntime->month, ntime->day,
		 ntime->hour, ntime->min, ntime->sec,
		 tz_hours, tz_minutes);
	(void)publish_tracker_string_state("net_time", net_time);
	(void)publish_tracker_int_state("tz", tz_hours);
	if (!app_tz_qh_deleted) {
		delete_status = delete_tracker_state("tz_qh");
		LOG_DBG("tz_qh state delete status=%d", delete_status);
		app_tz_qh_deleted = true;
	}
	LOG_DBG("Network time published: %s tz=%d", net_time, tz_hours);
}

static void prune_optional_sensor_state(void)
{
	enum golioth_status pressure_status;
	enum golioth_status pressure_avail_status;
	enum golioth_status light_status;
	enum golioth_status light_avail_status;

	if (optional_sensor_state_pruned || client == NULL) {
		return;
	}

	pressure_status = delete_tracker_state("pressure_hpa_x10");
	pressure_avail_status = delete_tracker_state("pressure_avail");
	light_status = delete_tracker_state("light_lux");
	light_avail_status = delete_tracker_state("light_avail");
	LOG_DBG("Optional sensor state pruned status pressure=(%d,%d) light=(%d,%d)",
		pressure_status, pressure_avail_status, light_status, light_avail_status);
	optional_sensor_state_pruned = true;
}

static void on_client_event(struct golioth_client *client,
			    enum golioth_client_event event,
			    void *arg)
{
	bool is_connected = (event == GOLIOTH_CLIENT_EVENT_CONNECTED);

	ARG_UNUSED(client);
	ARG_UNUSED(arg);

	if (is_connected) {
		k_sem_give(&connected);
		app_led_pattern(APP_LED_CONNECTED);
	}

	LOG_INF("Golioth client %s", is_connected ? "connected" : "disconnected");
}

static int app_extract_plausible_mv(const char *resp)
{
	const char *cursor = resp;

	while (cursor != NULL && *cursor != '\0') {
		char *endptr;
		long value;

		while (*cursor != '\0' && !isdigit((unsigned char)*cursor) &&
		       *cursor != '-') {
			cursor++;
		}

		if (*cursor == '\0') {
			break;
		}

		value = strtol(cursor, &endptr, 10);
		if (endptr == cursor) {
			cursor++;
			continue;
		}

		if (value >= 2500L && value <= 5000L) {
			return (int)value;
		}

		cursor = endptr;
	}

	return -1;
}

static int read_battery_mv(void)
{
	char resp[256] = {0};
	int charge_state = 0;
	int charge_pct = 0;
	int batt_mv = 0;
	const char *cbc;
	int ret;

	ret = bg9x_at_cmd_passthrough("AT+QADC=0", resp, sizeof(resp));
	if (ret == 0 && strstr(resp, "+QADC:") != NULL) {
		batt_mv = app_extract_plausible_mv(resp);
		if (batt_mv >= 0) {
			last_batt_pct = -1;
			last_batt_mv = batt_mv;
			LOG_DBG("QADC mv=%d", batt_mv);
			return batt_mv;
		}
	}

	ret = bg9x_at_cmd_passthrough("AT+CBC", resp, sizeof(resp));
	if (ret == 0 && strstr(resp, "+CBC:") != NULL) {
		cbc = strstr(resp, "+CBC:");
		if ((cbc != NULL &&
		     (sscanf(cbc, "+CBC: %d,%d,%d", &charge_state, &charge_pct, &batt_mv) == 3 ||
		      sscanf(cbc, "+CBC:%d,%d,%d", &charge_state, &charge_pct, &batt_mv) == 3)) ||
		    (batt_mv = app_extract_plausible_mv(resp)) >= 0) {
			last_batt_pct = (charge_pct >= 0 && charge_pct <= 100) ? charge_pct : -1;
			last_batt_mv = batt_mv;
			LOG_DBG("CBC state=%d pct=%d mv=%d", charge_state, charge_pct,
				batt_mv);
			return batt_mv;
		}
	}

	last_batt_pct = -1;
	LOG_WRN("battery read failed qadc/cbc ret=%d resp=%s", ret, resp);
	return -1;
}

static void app_debug_seed_tracker_cache(int counter)
{
	struct bg9x_cell_info cell = {0};
	struct bg9x_network_time ntime = {0};

	if (last_batt_mv < 0 || last_batt_pct < 0) {
		(void)read_battery_mv();
	}

	if (!app_last_valid_cell_info_present) {
		app_refresh_connected_network_snapshot(counter, &cell, &ntime);
	}
}

static bool iface_has_ipv4(struct net_if *iface)
{
	struct net_if_ipv4 *ipv4;

	if (iface == NULL) {
		return false;
	}

	ipv4 = iface->config.ip.ipv4;
	if (ipv4 == NULL) {
		return false;
	}

	for (int i = 0; i < NET_IF_MAX_IPV4_ADDR; i++) {
		if (ipv4->unicast[i].ipv4.is_used &&
		    ipv4->unicast[i].ipv4.address.in_addr.s_addr != 0) {
			return true;
		}
	}

	return false;
}

static void log_iface_state(const char *tag)
{
	struct net_if *iface = net_if_get_default();

	if (iface == NULL) {
		LOG_WRN("net_if[%s]: no default iface", tag);
		return;
	}

	LOG_INF("net_if[%s]: up=%d carrier=%d dormant=%d ipv4=%d",
		tag,
		net_if_is_up(iface),
		net_if_is_carrier_ok(iface),
		net_if_is_dormant(iface),
		iface_has_ipv4(iface));
}

#if defined(CONFIG_MODEM_CELLULAR)
static void modem_diag_log_any_match(struct modem_chat *chat, char **argv, uint16_t argc,
				     void *user_data)
{
	ARG_UNUSED(chat);
	ARG_UNUSED(user_data);

	if (argc == 2) {
		LOG_INF("AT<< %s", argv[1]);
	}
}

static void modem_diag_log_match(struct modem_chat *chat, char **argv, uint16_t argc,
				 void *user_data)
{
	ARG_UNUSED(chat);
	ARG_UNUSED(user_data);

	if (argc == 1) {
		LOG_INF("AT<< %s", argv[0]);
	}
}

MODEM_CHAT_MATCHES_DEFINE(modem_diag_abort_matches,
			  MODEM_CHAT_MATCH("ERROR", "", modem_diag_log_match));

static void modem_diag_script_callback(struct modem_chat *chat,
				       enum modem_chat_script_result result,
				       void *user_data)
{
	struct modem_diag_ctx *diag = user_data;

	ARG_UNUSED(chat);

	diag->last_result = result;
	atomic_clear_bit(&diag->state, MODEM_DIAG_BUSY_BIT);
	k_sem_give(&diag->done_sem);
}

MODEM_CHAT_SCRIPT_DEFINE(modem_diag_script,
			 modem_diag.script_chat,
			 modem_diag_abort_matches,
			 modem_diag_script_callback,
			 MODEM_DIAG_TIMEOUT_SECONDS);

static void modem_diag_pipe_callback(struct modem_pipe *pipe, enum modem_pipe_event event,
				     void *user_data)
{
	ARG_UNUSED(pipe);
	ARG_UNUSED(user_data);

	if (event == MODEM_PIPE_EVENT_OPENED) {
		LOG_INF("AT pipe opened");
		k_work_submit(&modem_diag.attach_chat_work);
	}
}

static void modem_diag_pipelink_callback(struct modem_pipelink *link,
					 enum modem_pipelink_event event,
					 void *user_data)
{
	ARG_UNUSED(link);
	ARG_UNUSED(user_data);

	switch (event) {
	case MODEM_PIPELINK_EVENT_CONNECTED:
		LOG_INF("AT pipe connected");
		k_work_submit(&modem_diag.open_pipe_work);
		break;

	case MODEM_PIPELINK_EVENT_DISCONNECTED:
		LOG_WRN("AT pipe disconnected");
		k_work_submit(&modem_diag.release_chat_work);
		break;

	default:
		break;
	}
}

static void modem_diag_open_pipe_handler(struct k_work *work)
{
	ARG_UNUSED(work);

	modem_pipe_attach(modem_pipelink_get_pipe(modem_diag.pipelink),
			  modem_diag_pipe_callback,
			  NULL);
	modem_pipe_open_async(modem_pipelink_get_pipe(modem_diag.pipelink));
}

static void modem_diag_attach_chat_handler(struct k_work *work)
{
	ARG_UNUSED(work);

	modem_chat_attach(&modem_diag.chat, modem_pipelink_get_pipe(modem_diag.pipelink));
	atomic_set_bit(&modem_diag.state, MODEM_DIAG_ATTACHED_BIT);
	k_sem_give(&modem_diag.attached_sem);
	printk("AT chat attached\n");
	LOG_INF("AT chat attached");
}

static void modem_diag_release_chat_handler(struct k_work *work)
{
	ARG_UNUSED(work);

	modem_chat_release(&modem_diag.chat);
	atomic_clear_bit(&modem_diag.state, MODEM_DIAG_ATTACHED_BIT);
	LOG_INF("AT chat released");
}

static void modem_diag_init(void)
{
	const struct modem_chat_config chat_config = {
		.receive_buf = modem_diag.chat_receive_buf,
		.receive_buf_size = sizeof(modem_diag.chat_receive_buf),
		.delimiter = "\r",
		.delimiter_size = sizeof("\r") - 1,
		.filter = "\n",
		.filter_size = sizeof("\n") - 1,
		.argv = modem_diag.chat_argv_buf,
		.argv_size = ARRAY_SIZE(modem_diag.chat_argv_buf),
	};

	k_work_init(&modem_diag.open_pipe_work, modem_diag_open_pipe_handler);
	k_work_init(&modem_diag.attach_chat_work, modem_diag_attach_chat_handler);
	k_work_init(&modem_diag.release_chat_work, modem_diag_release_chat_handler);
	k_sem_init(&modem_diag.attached_sem, 0, 1);
	k_sem_init(&modem_diag.done_sem, 0, 1);

	modem_chat_init(&modem_diag.chat, &chat_config);
	modem_chat_match_init(&modem_diag.script_matches[0]);
	modem_chat_match_set_match(&modem_diag.script_matches[0], "");
	modem_chat_match_set_separators(&modem_diag.script_matches[0], "");
	modem_chat_match_set_callback(&modem_diag.script_matches[0], modem_diag_log_any_match);
	modem_chat_match_set_partial(&modem_diag.script_matches[0], true);
	modem_chat_match_enable_wildcards(&modem_diag.script_matches[0], false);

	modem_chat_match_init(&modem_diag.script_matches[1]);
	modem_chat_match_set_match(&modem_diag.script_matches[1], "OK");
	modem_chat_match_set_separators(&modem_diag.script_matches[1], "");
	modem_chat_match_set_callback(&modem_diag.script_matches[1], modem_diag_log_match);
	modem_chat_match_set_partial(&modem_diag.script_matches[1], false);
	modem_chat_match_enable_wildcards(&modem_diag.script_matches[1], false);

	modem_chat_script_chat_init(modem_diag.script_chat);
	modem_chat_script_chat_set_response_matches(modem_diag.script_chat,
						    modem_diag.script_matches,
						    ARRAY_SIZE(modem_diag.script_matches));
	modem_chat_script_chat_set_timeout(modem_diag.script_chat,
					   MODEM_DIAG_TIMEOUT_SECONDS);
	modem_pipelink_attach(modem_diag.pipelink, modem_diag_pipelink_callback, NULL);
}

static bool modem_diag_ready(void)
{
	return atomic_test_bit(&modem_diag.state, MODEM_DIAG_ATTACHED_BIT);
}

static bool modem_diag_wait_ready(int timeout_seconds)
{
	if (modem_diag_ready()) {
		return true;
	}

	return k_sem_take(&modem_diag.attached_sem, K_SECONDS(timeout_seconds)) == 0;
}

static int modem_diag_run_cmd(const char *cmd, const char *expected)
{
	int ret;

	if (!modem_diag_ready()) {
		LOG_WRN("AT pipe not ready for '%s'", cmd);
		return -EAGAIN;
	}

	if (atomic_test_and_set_bit(&modem_diag.state, MODEM_DIAG_BUSY_BIT)) {
		LOG_WRN("AT pipe busy, skipping '%s'", cmd);
		return -EBUSY;
	}

	strncpy(modem_diag.request_buf, cmd, sizeof(modem_diag.request_buf) - 1);
	modem_diag.request_buf[sizeof(modem_diag.request_buf) - 1] = '\0';
	strncpy(modem_diag.match_buf, expected, sizeof(modem_diag.match_buf) - 1);
	modem_diag.match_buf[sizeof(modem_diag.match_buf) - 1] = '\0';
	modem_diag.last_result = MODEM_CHAT_SCRIPT_RESULT_ABORT;
	k_sem_reset(&modem_diag.done_sem);

	ret = modem_chat_script_chat_set_request(modem_diag.script_chat, modem_diag.request_buf);
	if (ret < 0) {
		atomic_clear_bit(&modem_diag.state, MODEM_DIAG_BUSY_BIT);
		return ret;
	}

	ret = modem_chat_match_set_match(&modem_diag.script_matches[1], modem_diag.match_buf);
	if (ret < 0) {
		atomic_clear_bit(&modem_diag.state, MODEM_DIAG_BUSY_BIT);
		return ret;
	}

	LOG_INF("AT>> %s", cmd);
	printk("AT>> %s\n", cmd);

	ret = modem_chat_run_script_async(&modem_diag.chat, &modem_diag_script);
	if (ret < 0) {
		atomic_clear_bit(&modem_diag.state, MODEM_DIAG_BUSY_BIT);
		return ret;
	}

	if (k_sem_take(&modem_diag.done_sem, K_SECONDS(MODEM_DIAG_TIMEOUT_SECONDS)) != 0) {
		LOG_WRN("AT timeout for '%s'", cmd);
		atomic_clear_bit(&modem_diag.state, MODEM_DIAG_BUSY_BIT);
		return -ETIMEDOUT;
	}

	if (modem_diag.last_result != MODEM_CHAT_SCRIPT_RESULT_SUCCESS) {
		LOG_WRN("AT failed for '%s' (result=%d)", cmd, modem_diag.last_result);
		return -EIO;
	}

	return 0;
}

static void modem_diag_snapshot(const char *tag)
{
	printk("modem snapshot [%s]\n", tag);
	LOG_INF("modem snapshot [%s]", tag);
	(void)modem_diag_run_cmd("AT", "OK");
	(void)modem_diag_run_cmd("AT+CPSMS?", "OK");
	(void)modem_diag_run_cmd("AT+QPSMS?", "OK");
	(void)modem_diag_run_cmd("AT+QCSCON?", "OK");
	(void)modem_diag_run_cmd("AT+CEREG?", "OK");
}

static void modem_diag_configure_psm(void)
{
	LOG_INF("diag pipe ready; skipping redundant boot-time PSM override (driver owns PSM config)");
}
#else
static void modem_diag_init(void)
{
}

static bool modem_diag_wait_ready(int timeout_seconds)
{
	ARG_UNUSED(timeout_seconds);
	return false;
}

static bool modem_diag_ready(void)
{
	return false;
}

static int modem_diag_run_cmd(const char *cmd, const char *expected)
{
	ARG_UNUSED(cmd);
	ARG_UNUSED(expected);
	return -ENOTSUP;
}

static void modem_diag_snapshot(const char *tag)
{
	ARG_UNUSED(tag);
}

static void modem_diag_configure_psm(void)
{
}
#endif

static void read_sensors(void)
{
	struct sensor_value val[3];
	int err;

	/* Accelerometer: LIS3DH */
	if (device_is_ready(sensor_accel)) {
		err = sensor_sample_fetch(sensor_accel);
		if (err == 0) {
			sensor_channel_get(sensor_accel, SENSOR_CHAN_ACCEL_X, &val[0]);
			sensor_channel_get(sensor_accel, SENSOR_CHAN_ACCEL_Y, &val[1]);
			sensor_channel_get(sensor_accel, SENSOR_CHAN_ACCEL_Z, &val[2]);
			last_accel_x = (int32_t)(sensor_value_to_double(&val[0]) * 1000.0 / 9.80665);
			last_accel_y = (int32_t)(sensor_value_to_double(&val[1]) * 1000.0 / 9.80665);
			last_accel_z = (int32_t)(sensor_value_to_double(&val[2]) * 1000.0 / 9.80665);
			LOG_DBG("accel x=%d y=%d z=%d mg", last_accel_x, last_accel_y, last_accel_z);
		} else {
			LOG_WRN("accel fetch failed: %d", err);
		}
	} else {
		LOG_WRN("accel device not ready");
	}

	/* Temperature + Humidity: SHTC3 */
	if (device_is_ready(sensor_shtc3)) {
		err = sensor_sample_fetch(sensor_shtc3);
		if (err == 0) {
			sensor_channel_get(sensor_shtc3, SENSOR_CHAN_AMBIENT_TEMP, &val[0]);
			sensor_channel_get(sensor_shtc3, SENSOR_CHAN_HUMIDITY, &val[1]);
			double temp_c = sensor_value_to_double(&val[0]);
			double humidity = sensor_value_to_double(&val[1]);
			last_temp_c10 = (int32_t)(temp_c * 10.0);
			last_humid_c10 = (int32_t)(humidity * 10.0);
			LOG_DBG("temp=%.1f C humidity=%.1f %%", temp_c, humidity);
		} else {
			LOG_WRN("shtc3 fetch failed: %d", err);
		}
	} else {
		LOG_WRN("shtc3 device not ready");
	}

	if (sensor_press != NULL && device_is_ready(sensor_press)) {
		pressure_sensor_available = true;
		err = sensor_sample_fetch(sensor_press);
		if (err == 0) {
			sensor_channel_get(sensor_press, SENSOR_CHAN_PRESS, &val[0]);
			last_pressure_hpa10 =
				(int32_t)(sensor_value_to_double(&val[0]) * 100.0);
			LOG_DBG("pressure=%d.%d hPa",
				last_pressure_hpa10 / 10,
				abs(last_pressure_hpa10) % 10);
		} else {
			LOG_WRN("lps22hb fetch failed: %d", err);
		}
	} else {
		pressure_sensor_available = false;
		last_pressure_hpa10 = 0;
		if (!pressure_sensor_absent_logged) {
			LOG_INF("lps22hb not present on this board; pruning pressure state");
			pressure_sensor_absent_logged = true;
		}
	}

	if (sensor_light != NULL && device_is_ready(sensor_light)) {
		light_sensor_available = true;
		err = sensor_sample_fetch(sensor_light);
		if (err == 0) {
			sensor_channel_get(sensor_light, SENSOR_CHAN_LIGHT, &val[0]);
			last_light_lux = (int32_t)(sensor_value_to_double(&val[0]) + 0.5);
			LOG_DBG("light=%d lux", last_light_lux);
		} else {
			LOG_WRN("opt3001 fetch failed: %d", err);
		}
	} else {
		light_sensor_available = false;
		last_light_lux = 0;
		if (!light_sensor_absent_logged) {
			LOG_INF("opt3001 not present on this board; pruning light state");
			light_sensor_absent_logged = true;
		}
	}
}

/* Shell command: send raw AT command to BG95 modem */
static int cmd_at(const struct shell *sh, size_t argc, char **argv)
{
	char cmd[128];
	char resp[512];
	size_t offset = 0;
	int ret;

	if (argc < 2) {
		shell_print(sh, "Usage: at <AT command>");
		return -EINVAL;
	}

	for (int i = 1; i < (int)argc && offset < sizeof(cmd) - 1; i++) {
		if (i > 1 && offset < sizeof(cmd) - 1) {
			cmd[offset++] = ' ';
		}
		size_t len = strlen(argv[i]);

		if (offset + len >= sizeof(cmd)) {
			break;
		}
		memcpy(cmd + offset, argv[i], len);
		offset += len;
	}
	cmd[offset] = '\0';

	ret = bg9x_at_cmd_passthrough(cmd, resp, sizeof(resp));
	if (resp[0] != '\0') {
		shell_print(sh, "%s", resp);
	}
	shell_print(sh, "%s (ret=%d)", ret == 0 ? "OK" : "ERROR", ret);
	return 0;
}

/* Shell command: read cell info */
static int cmd_cell(const struct shell *sh, size_t argc, char **argv)
{
	struct bg9x_cell_info info;
	int ret;

	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	ret = bg9x_cell_info_get(&info);
	if (ret < 0) {
		shell_print(sh, "cell info failed: %d", ret);
		return ret;
	}

	if (!info.valid) {
		shell_print(sh, "cell info: no data");
		return 0;
	}

	shell_print(sh, "RAT=%s MCC=%u MNC=%u cellID=%x PCI=%u",
		    info.rat, info.mcc, info.mnc, info.cell_id, info.pci);
	shell_print(sh, "EARFCN=%u band=%u TAC=%x",
		    info.earfcn, info.band, info.tac);
	shell_print(sh, "RSRP=%d RSRQ=%d RSSI=%d SINR=%d",
		    info.rsrp, info.rsrq, info.rssi, info.sinr);
	return 0;
}

static void app_modem_usage_cycle_snapshot(int counter, const char *phase, bool baseline)
{
	struct bg9x_data_usage usage;
	uint64_t stage_tx = 0U;
	uint64_t stage_rx = 0U;
	uint64_t total_tx = 0U;
	uint64_t total_rx = 0U;
	int ret;

	ret = bg9x_data_usage_get(&usage);
	if (ret < 0) {
		LOG_INF("#%d qgdcnt(%s) read failed ret=%d", counter, phase, ret);
		if (baseline) {
			app_cycle_data_usage_valid = false;
		}
		return;
	}

	if (!usage.valid) {
		LOG_INF("#%d qgdcnt(%s) no data", counter, phase);
		if (baseline) {
			app_cycle_data_usage_valid = false;
		}
		return;
	}

	LOG_INF("#%d qgdcnt(%s) tx=%llu rx=%llu",
		counter, phase,
		(unsigned long long)usage.tx_bytes,
		(unsigned long long)usage.rx_bytes);

	if (baseline) {
		app_cycle_data_usage_start = usage;
		app_cycle_data_usage_last = usage;
		app_cycle_data_usage_last_phase = phase;
		app_cycle_data_usage_valid = true;
		return;
	}

	if (!app_cycle_data_usage_valid) {
		LOG_INF("#%d qgdcnt(%s) delta unavailable: no baseline", counter, phase);
		return;
	}

	if (usage.tx_bytes < app_cycle_data_usage_start.tx_bytes ||
	    usage.rx_bytes < app_cycle_data_usage_start.rx_bytes ||
	    usage.tx_bytes < app_cycle_data_usage_last.tx_bytes ||
	    usage.rx_bytes < app_cycle_data_usage_last.rx_bytes) {
		LOG_INF("#%d qgdcnt(%s) delta unavailable: counter reset start_tx=%llu start_rx=%llu last_tx=%llu last_rx=%llu end_tx=%llu end_rx=%llu",
			counter, phase,
			(unsigned long long)app_cycle_data_usage_start.tx_bytes,
			(unsigned long long)app_cycle_data_usage_start.rx_bytes,
			(unsigned long long)app_cycle_data_usage_last.tx_bytes,
			(unsigned long long)app_cycle_data_usage_last.rx_bytes,
			(unsigned long long)usage.tx_bytes,
			(unsigned long long)usage.rx_bytes);
		return;
	}

	stage_tx = usage.tx_bytes - app_cycle_data_usage_last.tx_bytes;
	stage_rx = usage.rx_bytes - app_cycle_data_usage_last.rx_bytes;
	total_tx = usage.tx_bytes - app_cycle_data_usage_start.tx_bytes;
	total_rx = usage.rx_bytes - app_cycle_data_usage_start.rx_bytes;

	LOG_INF("#%d qgdcnt(%s) stage(%s) tx=%llu rx=%llu total=%llu since-start tx=%llu rx=%llu total=%llu",
		counter, phase,
		app_cycle_data_usage_last_phase != NULL ? app_cycle_data_usage_last_phase : "baseline",
		(unsigned long long)stage_tx,
		(unsigned long long)stage_rx,
		(unsigned long long)(stage_tx + stage_rx),
		(unsigned long long)total_tx,
		(unsigned long long)total_rx,
		(unsigned long long)(total_tx + total_rx));

	app_cycle_data_usage_last = usage;
	app_cycle_data_usage_last_phase = phase;
}

static int app_debug_ensure_client_created(void)
{
	if (client != NULL) {
		return 0;
	}

	if (app_client_config == NULL) {
		return -EINVAL;
	}

	client = golioth_client_create(app_client_config);
	if (client == NULL) {
		LOG_ERR("debug client create failed");
		return -ENOMEM;
	}

	golioth_client_register_event_callback(client, on_client_event, NULL);
	LOG_INF("Golioth client created");
	return 0;
}

static int app_debug_connect_client(uint32_t timeout_seconds)
{
	int ret;

	ret = app_debug_ensure_client_created();
	if (ret < 0) {
		return ret;
	}

	if (golioth_client_is_connected(client)) {
		LOG_INF("debug connectonly: client already connected");
		return 0;
	}

	app_debug_seed_tracker_cache((int)app_debug_probe_counter + 1);
	app_debug_probe_counter++;
	app_modem_usage_cycle_snapshot((int)app_debug_probe_counter, "debug-pre-connect", true);
	k_sem_reset(&connected);
	golioth_client_start(client);
	LOG_INF("debug connectonly: waiting %us for connect", timeout_seconds);
	ret = k_sem_take(&connected, K_SECONDS(timeout_seconds));
	if (ret < 0) {
		LOG_WRN("debug connectonly timeout ret=%d", ret);
		return ret;
	}

	app_modem_usage_cycle_snapshot((int)app_debug_probe_counter, "debug-post-connect", false);
	return 0;
}

static int app_debug_disconnect_client(void)
{
	if (client == NULL) {
		return 0;
	}

	golioth_client_stop(client);
	return 0;
}

static int app_debug_send_probe(size_t payload_len)
{
	uint8_t payload[8];
	enum golioth_status status;

	if (payload_len == 0U || payload_len > sizeof(payload)) {
		return -EINVAL;
	}

	if (client == NULL || !golioth_client_is_connected(client)) {
		return -ENOTCONN;
	}

	for (size_t i = 0; i < payload_len; i++) {
		payload[i] = (uint8_t)('A' + (i % 26U));
	}

	status = golioth_stream_set_sync(client, app_debug_probe_path,
					 GOLIOTH_CONTENT_TYPE_OCTET_STREAM,
					 payload, payload_len,
					 TRACKER_STREAM_TIMEOUT_SECONDS);
	LOG_INF("debug probe status=%d len=%u path=%s",
		status, (unsigned int)payload_len, app_debug_probe_path);
	app_modem_usage_cycle_snapshot((int)app_debug_probe_counter, "debug-post-probe", false);

	if (status == GOLIOTH_OK || status == GOLIOTH_ERR_TIMEOUT) {
		return 0;
	}

	return -EIO;
}

static bool app_debug_build_cached_fix(struct bg9x_gps_fix *fix)
{
	if (fix == NULL) {
		return false;
	}

	memset(fix, 0, sizeof(*fix));
	if (app_gps_lat_e6 == 0 && app_gps_lon_e6 == 0) {
		return false;
	}

	fix->valid = true;
	fix->fix_type = 3U;
	fix->latitude = ((double)app_gps_lat_e6) / 1000000.0;
	fix->longitude = ((double)app_gps_lon_e6) / 1000000.0;
	fix->altitude = ((float)app_gps_alt_cm) / 100.0f;
	fix->speed_kmh = ((float)app_gps_speed_x10) / 10.0f;
	fix->course = ((float)app_gps_course_x10) / 10.0f;
	fix->sats_in_use = (uint8_t)MIN(app_gps_sats, UINT8_MAX);
	fix->fix_time_ms = app_gps_fixtime_ms;

	return true;
}

static int app_debug_send_tracker_from_cache(void)
{
	struct bg9x_gps_fix fix = {0};
	struct bg9x_cell_info cell = {0};
	struct bg9x_network_time ntime = {0};
	uint32_t next_wake_seconds;
	enum tracker_publish_result result;
	int batt_mv = last_batt_mv;

	if (client == NULL || !golioth_client_is_connected(client)) {
		LOG_WRN("debug tracker send aborted: client not connected");
		return -ENOTCONN;
	}

	if (!app_debug_build_cached_fix(&fix)) {
		LOG_WRN("debug tracker send skipped: no cached GPS fix");
		return -ENODATA;
	}

	if (app_last_valid_cell_info_present) {
		cell = app_last_valid_cell_info;
	}
	if (app_last_valid_network_time_present) {
		ntime = app_last_valid_network_time;
	}
	LOG_INF("debug tracker send: using cached batt=%d cell_valid=%u batt_pct=%d",
		batt_mv, cell.valid ? 1U : 0U, last_batt_pct);
	next_wake_seconds = app_next_wake_seconds(false);
	LOG_INF("debug tracker send: publishing cached tracker next_wake=%u", next_wake_seconds);
	result = publish_tracker_stream_record((int)app_debug_probe_counter, false, batt_mv,
					       next_wake_seconds, &fix, &cell, &ntime);
	LOG_INF("debug tracker send: publish result=%d", result);
	app_modem_usage_cycle_snapshot((int)app_debug_probe_counter, "debug-post-tracker", false);

	if (result == TRACKER_PUBLISH_SENT || result == TRACKER_PUBLISH_TIMEOUT) {
		return 0;
	}

	return -EIO;
}

static void app_debug_sendtracker_work_handler(struct k_work *work)
{
	int ret;

	ARG_UNUSED(work);

	app_debug_probe_counter++;
	LOG_INF("debug sendtracker worker: begin");
	if (client == NULL || !golioth_client_is_connected(client)) {
		LOG_INF("debug sendtracker worker: connecting client first");
		ret = app_debug_connect_client(APP_DEBUG_CONNECT_TIMEOUT_SECONDS);
		if (ret < 0) {
			LOG_WRN("debug sendtracker worker: connect failed ret=%d", ret);
			goto out;
		}
	} else {
		LOG_INF("debug sendtracker worker: forcing modem socket reopen under live DTLS session");
		ret = bg9x_connected_socket_reopen();
		if (ret < 0) {
			LOG_WRN("debug sendtracker worker: socket reopen failed ret=%d", ret);
			goto out;
		}
	}

	ret = app_debug_send_tracker_from_cache();

out:
	LOG_INF("debug sendtracker worker: done ret=%d path=%s",
		ret, tracker_stream_path());
	atomic_clear(&app_debug_sendtracker_busy);
}

static void app_debug_sendtracker_autorun_work_handler(struct k_work *work)
{
	int ret;

	ARG_UNUSED(work);

	if (!app_manual_cycle_mode_enabled()) {
		return;
	}

	LOG_INF("debug autorun: running delayed inline connect->reopen->tracker self-test");
	app_debug_probe_counter++;
	ret = app_debug_connect_client(APP_DEBUG_CONNECT_TIMEOUT_SECONDS);
	LOG_INF("debug autorun: connect ret=%d", ret);
	if (ret != 0) {
		return;
	}

	ret = bg9x_connected_socket_reopen();
	LOG_INF("debug autorun: socket reopen ret=%d", ret);
	if (ret != 0) {
		return;
	}

	ret = app_debug_send_tracker_from_cache();
	LOG_INF("debug autorun: tracker send ret=%d", ret);
}

static void app_debug_resumetracker_work_handler(struct k_work *work)
{
	uint32_t idle_seconds;
	uint32_t remaining_runs;
	uint32_t run_index = 0U;
	int ret;

	ARG_UNUSED(work);

	idle_seconds = app_debug_resumetracker_idle_seconds;
	remaining_runs = app_debug_resumetracker_count;

	LOG_INF("debug resumetracker worker: begin idle=%us count=%u",
		idle_seconds, remaining_runs);
	ret = 0;

	while (remaining_runs-- > 0U) {
		run_index++;
		LOG_INF("debug resumetracker worker: run %u/%u",
			run_index, app_debug_resumetracker_count);

		if (client == NULL || !golioth_client_is_connected(client)) {
			LOG_INF("debug resumetracker worker: connecting client first");
			ret = app_debug_connect_client(30U);
			if (ret < 0) {
				LOG_WRN("debug resumetracker worker: connect failed ret=%d",
					ret);
				goto out;
			}
		}

		ret = bg9x_psm_set_interval(true, idle_seconds + 60U);
		if (ret < 0) {
			LOG_WRN("debug resumetracker worker: set psm interval failed ret=%d",
				ret);
		}

		app_debug_log_modemsnap("debug-resumetracker-pre-idle", false);
		LOG_INF("debug resumetracker worker: idling connected session for %us",
			idle_seconds);
		k_sleep(K_SECONDS(idle_seconds));
		app_debug_log_modemsnap("debug-resumetracker-post-idle", false);
		app_debug_probe_counter++;
		ret = app_debug_prepare_resume_socket("debug-resumetracker");
		if (ret < 0) {
			goto out;
		}
		app_modem_usage_cycle_snapshot((int)app_debug_probe_counter,
					       "debug-post-recover-pre-send", false);
		LOG_INF("debug resumetracker worker: sending cached tracker via send-path recovery");
		ret = app_debug_send_tracker_from_cache();
		if (ret < 0) {
			goto out;
		}
	}

out:
	LOG_INF("debug resumetracker worker: done ret=%d path=%s runs=%u/%u",
		ret, tracker_stream_path(), run_index, app_debug_resumetracker_count);
	atomic_clear(&app_debug_resumetracker_busy);
}

static int cmd_qgdcnt(const struct shell *sh, size_t argc, char **argv)
{
	struct bg9x_data_usage usage;
	uint64_t delta_tx = 0U;
	uint64_t delta_rx = 0U;
	bool have_delta = false;
	int ret;

	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	ret = bg9x_data_usage_get(&usage);
	if (ret < 0) {
		shell_print(sh, "qgdcnt failed: %d", ret);
		return ret;
	}

	if (!usage.valid) {
		shell_print(sh, "qgdcnt: no data");
		return 0;
	}

	if (app_last_data_usage_valid &&
	    usage.tx_bytes >= app_last_data_usage.tx_bytes &&
	    usage.rx_bytes >= app_last_data_usage.rx_bytes) {
		delta_tx = usage.tx_bytes - app_last_data_usage.tx_bytes;
		delta_rx = usage.rx_bytes - app_last_data_usage.rx_bytes;
		have_delta = true;
	}

	shell_print(sh, "qgdcnt tx=%llu rx=%llu",
		    (unsigned long long)usage.tx_bytes,
		    (unsigned long long)usage.rx_bytes);
	if (have_delta) {
		shell_print(sh, "qgdcnt delta tx=%llu rx=%llu",
			    (unsigned long long)delta_tx,
			    (unsigned long long)delta_rx);
	} else if (app_last_data_usage_valid) {
		shell_print(sh, "qgdcnt delta unavailable (counter reset or decreased)");
	} else {
		shell_print(sh, "qgdcnt baseline recorded");
	}

	app_last_data_usage = usage;
	app_last_data_usage_valid = true;
	return 0;
}

static int cmd_modemsnap(const struct shell *sh, size_t argc, char **argv)
{
	struct bg9x_runtime_snapshot snapshot;
	bool probe_at = false;
	int ret;

	if (argc > 2) {
		shell_error(sh, "usage: modemsnap [wake]");
		return -EINVAL;
	}

	if (argc == 2) {
		if (strcmp(argv[1], "wake") != 0) {
			shell_error(sh, "usage: modemsnap [wake]");
			return -EINVAL;
		}
		probe_at = true;
	}

	ret = bg9x_runtime_snapshot_get(&snapshot, probe_at);
	if (ret < 0) {
		shell_error(sh, "modemsnap failed: %d", ret);
		return ret;
	}

	shell_print(sh,
		    "modemsnap probe=%d at_ready=%d at_ret=%d setup=%d wake_pending=%d dns=%d pdp=%d gps=%d alloc=%u conn=%u rssi=%d",
		    probe_at ? 1 : 0, snapshot.at_ready ? 1 : 0, snapshot.at_probe_ret,
		    snapshot.setup_complete ? 1 : 0, snapshot.runtime_wake_pending ? 1 : 0,
		    snapshot.dns_query_active ? 1 : 0, snapshot.pdp_active ? 1 : 0,
		    snapshot.gps_active ? 1 : 0, snapshot.allocated_sockets,
		    snapshot.connected_sockets, snapshot.mdm_rssi);
	shell_print(sh, "lines status=%d ap_ready=%d psm=%d ri=%d",
		    snapshot.status_gpio, snapshot.ap_ready_gpio,
		    snapshot.psm_gpio, snapshot.ri_gpio);
	return 0;
}

static int cmd_uart1probe(const struct shell *sh, size_t argc, char **argv)
{
	char line[96];
	const char *tag = "manual";

	if (argc > 2) {
		shell_error(sh, "usage: uart1probe [tag]");
		return -EINVAL;
	}

	if (argc == 2 && argv[1] != NULL && argv[1][0] != '\0') {
		tag = argv[1];
	}

	snprintk(line, sizeof(line), "UART1 PROBE tag=%s build=%s boot=%u",
		 tag, APP_DEBUG_BUILD_TAG, app_boot_count);
	app_uart1_write_line(line);
	shell_print(sh, "uart1probe sent tag=%s", tag);
	return 0;
}

static void app_debug_log_modemsnap(const char *tag, bool probe_at)
{
	struct bg9x_runtime_snapshot snapshot;
	int ret;

	ret = bg9x_runtime_snapshot_get(&snapshot, probe_at);
	if (ret < 0) {
		LOG_WRN("%s modemsnap failed ret=%d", tag, ret);
		return;
	}

	LOG_INF("%s modemsnap probe=%d at_ready=%d at_ret=%d setup=%d wake_pending=%d dns=%d pdp=%d gps=%d alloc=%u conn=%u rssi=%d",
		tag, probe_at ? 1 : 0, snapshot.at_ready ? 1 : 0, snapshot.at_probe_ret,
		snapshot.setup_complete ? 1 : 0, snapshot.runtime_wake_pending ? 1 : 0,
		snapshot.dns_query_active ? 1 : 0, snapshot.pdp_active ? 1 : 0,
		snapshot.gps_active ? 1 : 0, snapshot.allocated_sockets,
		snapshot.connected_sockets, snapshot.mdm_rssi);
	LOG_INF("%s lines status=%d ap_ready=%d psm=%d ri=%d",
		tag, snapshot.status_gpio, snapshot.ap_ready_gpio,
		snapshot.psm_gpio, snapshot.ri_gpio);
}

static void app_debug_log_socket_state(const char *tag, bool prepare_on_timeout)
{
	char resp[192];
	int sock_id = -1;
	bool present = false;
	int ret;

	ret = bg9x_connected_socket_qistate_get(prepare_on_timeout, &sock_id, &present,
						 resp, sizeof(resp));
	if (ret < 0) {
		LOG_WRN("%s socket qistate failed ret=%d", tag, ret);
		return;
	}

	LOG_INF("%s socket id=%d present=%d resp=%s", tag, sock_id, present ? 1 : 0, resp);
}

static int app_debug_prepare_resume_socket(const char *tag)
{
	char resp[192];
	bool present = false;
	bool reopened = false;
	int ret;

	ret = bg9x_connected_socket_resume_prepare(&present, &reopened, resp,
						   sizeof(resp));
	if (ret < 0) {
		LOG_WRN("%s socket resume-prepare failed ret=%d", tag, ret);
		return ret;
	}

	LOG_INF("%s socket resume-prepare present=%d reopened=%d resp=%s",
		tag, present ? 1 : 0, reopened ? 1 : 0, resp);
	return 0;
}

static int cmd_connectonly(const struct shell *sh, size_t argc, char **argv)
{
	uint32_t timeout_seconds = app_cellular_timeout_seconds;
	int ret;

	if (argc > 2) {
		shell_error(sh, "usage: connectonly [timeout_s]");
		return -EINVAL;
	}

	if (argc == 2) {
		timeout_seconds = (uint32_t)strtoul(argv[1], NULL, 10);
		if (timeout_seconds == 0U) {
			shell_error(sh, "timeout must be > 0");
			return -EINVAL;
		}
	}

	ret = app_debug_connect_client(timeout_seconds);
	shell_print(sh, "connectonly ret=%d", ret);
	return ret;
}

static int cmd_disconnect(const struct shell *sh, size_t argc, char **argv)
{
	int ret;

	if (argc != 1) {
		shell_error(sh, "usage: disconnect");
		return -EINVAL;
	}

	ret = app_debug_disconnect_client();
	shell_print(sh, "disconnect ret=%d", ret);
	return ret;
}

static int cmd_resume_socket(const struct shell *sh, size_t argc, char **argv)
{
	int ret;

	if (argc != 1) {
		shell_error(sh, "usage: resume_socket");
		return -EINVAL;
	}

	ret = app_debug_prepare_resume_socket("debug-resume-socket");
	shell_print(sh, "resume_socket ret=%d", ret);
	return ret;
}

static int cmd_sendprobe(const struct shell *sh, size_t argc, char **argv)
{
	size_t payload_len = 1U;
	int ret;

	if (argc > 2) {
		shell_error(sh, "usage: sendprobe [1-8]");
		return -EINVAL;
	}

	if (argc == 2) {
		payload_len = (size_t)strtoul(argv[1], NULL, 10);
		if (payload_len == 0U || payload_len > 8U) {
			shell_error(sh, "payload length must be 1..8");
			return -EINVAL;
		}
	}

	ret = app_debug_send_probe(payload_len);
	shell_print(sh, "sendprobe ret=%d len=%u path=%s",
		    ret, (unsigned int)payload_len, app_debug_probe_path);
	return ret;
}

static int cmd_sendtracker(const struct shell *sh, size_t argc, char **argv)
{
	printk("sendtracker cmd enter\n");
	if (argc != 1) {
		shell_error(sh, "usage: sendtracker");
		return -EINVAL;
	}

	if (!atomic_cas(&app_debug_sendtracker_busy, 0, 1)) {
		printk("sendtracker cmd busy\n");
		shell_error(sh, "sendtracker already running");
		return -EBUSY;
	}

	printk("sendtracker cmd submit\n");
	k_work_submit_to_queue(&app_debug_workq, &app_debug_sendtracker_work);
	printk("sendtracker cmd submitted\n");
	shell_print(sh, "sendtracker queued path=%s", tracker_stream_path());
	return 0;
}

static int cmd_resumeprobe(const struct shell *sh, size_t argc, char **argv)
{
	uint32_t idle_seconds = 15U;
	size_t payload_len = 1U;
	int ret;

	if (argc > 3) {
		shell_error(sh, "usage: resumeprobe [idle_s] [1-8]");
		return -EINVAL;
	}

	if (argc >= 2) {
		idle_seconds = (uint32_t)strtoul(argv[1], NULL, 10);
		if (idle_seconds == 0U) {
			shell_error(sh, "idle_s must be > 0");
			return -EINVAL;
		}
	}

	if (argc == 3) {
		payload_len = (size_t)strtoul(argv[2], NULL, 10);
		if (payload_len == 0U || payload_len > 8U) {
			shell_error(sh, "payload length must be 1..8");
			return -EINVAL;
		}
	}

	if (client == NULL || !golioth_client_is_connected(client)) {
		shell_error(sh, "resumeprobe requires an active Golioth connection");
		return -ENOTCONN;
	}

	ret = bg9x_psm_set_interval(true, idle_seconds + 60U);
	if (ret < 0) {
		shell_error(sh, "failed to set psm interval: %d", ret);
		return ret;
	}

	app_debug_log_modemsnap("debug-resume-pre-idle", false);
	LOG_INF("debug resumeprobe: idling connected session for %us", idle_seconds);
	k_sleep(K_SECONDS(idle_seconds));
	app_debug_log_modemsnap("debug-resume-post-idle", false);
	app_debug_probe_counter++;
	ret = app_debug_prepare_resume_socket("debug-resume");
	if (ret < 0) {
		shell_print(sh, "resumeprobe ret=%d idle=%u len=%u path=%s",
			    ret, (unsigned int)idle_seconds, (unsigned int)payload_len,
			    app_debug_probe_path);
		return ret;
	}
	app_modem_usage_cycle_snapshot((int)app_debug_probe_counter,
				       "debug-post-recover-pre-send", false);
	ret = app_debug_send_probe(payload_len);
	shell_print(sh, "resumeprobe ret=%d idle=%u len=%u path=%s",
		    ret, (unsigned int)idle_seconds, (unsigned int)payload_len,
		    app_debug_probe_path);
	return ret;
}

static int cmd_resumeprobe_nopsm(const struct shell *sh, size_t argc, char **argv)
{
	uint32_t idle_seconds = 15U;
	size_t payload_len = 1U;
	int ret;

	if (argc > 3) {
		shell_error(sh, "usage: resumeprobe_nopsm [idle_s] [1-8]");
		return -EINVAL;
	}

	if (argc >= 2) {
		idle_seconds = (uint32_t)strtoul(argv[1], NULL, 10);
		if (idle_seconds == 0U) {
			shell_error(sh, "idle_s must be > 0");
			return -EINVAL;
		}
	}

	if (argc == 3) {
		payload_len = (size_t)strtoul(argv[2], NULL, 10);
		if (payload_len == 0U || payload_len > 8U) {
			shell_error(sh, "payload length must be 1..8");
			return -EINVAL;
		}
	}

	if (client == NULL || !golioth_client_is_connected(client)) {
		shell_error(sh, "resumeprobe_nopsm requires an active Golioth connection");
		return -ENOTCONN;
	}

	ret = bg9x_psm_set(false);
	if (ret < 0) {
		shell_error(sh, "failed to disable psm: %d", ret);
		return ret;
	}

	ret = bg9x_psm_set_interval(true, idle_seconds + 60U);
	if (ret < 0) {
		(void)bg9x_psm_set(true);
		shell_error(sh, "failed to set psm interval: %d", ret);
		return ret;
	}

	app_debug_log_modemsnap("debug-resume-nopsm-pre-idle", false);
	LOG_INF("debug resumeprobe_nopsm: idling connected session for %us with PSM disabled",
		idle_seconds);
	k_sleep(K_SECONDS(idle_seconds));
	app_debug_log_modemsnap("debug-resume-nopsm-post-idle", false);
	app_debug_probe_counter++;
	ret = app_debug_prepare_resume_socket("debug-resume-nopsm");
	if (ret < 0) {
		(void)bg9x_psm_set(true);
		shell_print(sh, "resumeprobe_nopsm ret=%d idle=%u len=%u path=%s",
			    ret, (unsigned int)idle_seconds, (unsigned int)payload_len,
			    app_debug_probe_path);
		return ret;
	}
	app_modem_usage_cycle_snapshot((int)app_debug_probe_counter,
				       "debug-post-recover-pre-send", false);
	ret = app_debug_send_probe(payload_len);
	(void)bg9x_psm_set(true);
	shell_print(sh, "resumeprobe_nopsm ret=%d idle=%u len=%u path=%s",
		    ret, (unsigned int)idle_seconds, (unsigned int)payload_len,
		    app_debug_probe_path);
	return ret;
}

static int cmd_resumetracker(const struct shell *sh, size_t argc, char **argv)
{
	uint32_t idle_seconds = 15U;
	uint32_t count = 1U;

	if (argc > 3) {
		shell_error(sh, "usage: resumetracker [idle_s] [count]");
		return -EINVAL;
	}

	if (argc >= 2) {
		idle_seconds = (uint32_t)strtoul(argv[1], NULL, 10);
		if (idle_seconds == 0U) {
			shell_error(sh, "idle_s must be > 0");
			return -EINVAL;
		}
	}

	if (argc == 3) {
		count = (uint32_t)strtoul(argv[2], NULL, 10);
		if (count == 0U) {
			shell_error(sh, "count must be > 0");
			return -EINVAL;
		}
	}

	printk("resumetracker cmd enter\n");
	if (!atomic_cas(&app_debug_resumetracker_busy, 0, 1)) {
		printk("resumetracker cmd busy\n");
		shell_error(sh, "resumetracker already running");
		return -EBUSY;
	}

	app_debug_resumetracker_idle_seconds = idle_seconds;
	app_debug_resumetracker_count = count;
	printk("resumetracker cmd queue psm interval=%u count=%u\n",
	       (unsigned int)(idle_seconds + 60U), (unsigned int)count);
	printk("resumetracker cmd submit idle=%u\n", (unsigned int)idle_seconds);
	k_work_submit_to_queue(&app_debug_workq, &app_debug_resumetracker_work);
	printk("resumetracker cmd submitted\n");
	shell_print(sh, "resumetracker queued idle=%u count=%u path=%s",
		    (unsigned int)idle_seconds, (unsigned int)count,
		    tracker_stream_path());
	return 0;
}

/* Shell command: read network time */
static int cmd_ntime(const struct shell *sh, size_t argc, char **argv)
{
	struct bg9x_network_time t;
	int ret;

	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	ret = bg9x_network_time_get(&t);
	if (ret < 0) {
		shell_print(sh, "ntime failed: %d", ret);
		return ret;
	}

	if (!t.valid) {
		shell_print(sh, "ntime: no data");
		return 0;
	}

	shell_print(sh, "%04u-%02u-%02uT%02u:%02u:%02u TZ=%+d quarter-hours",
		    t.year, t.month, t.day, t.hour, t.min, t.sec,
		    t.tz_quarter_hours);
	return 0;
}

/* Shell command: read sensors */
static int cmd_sensors(const struct shell *sh, size_t argc, char **argv)
{
	struct sensor_value val[3];
	int err;

	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	if (device_is_ready(sensor_accel)) {
		err = sensor_sample_fetch(sensor_accel);
		if (err == 0) {
			sensor_channel_get(sensor_accel, SENSOR_CHAN_ACCEL_X, &val[0]);
			sensor_channel_get(sensor_accel, SENSOR_CHAN_ACCEL_Y, &val[1]);
			sensor_channel_get(sensor_accel, SENSOR_CHAN_ACCEL_Z, &val[2]);
			int32_t x = (int32_t)(sensor_value_to_double(&val[0]) * 1000.0 / 9.80665);
			int32_t y = (int32_t)(sensor_value_to_double(&val[1]) * 1000.0 / 9.80665);
			int32_t z = (int32_t)(sensor_value_to_double(&val[2]) * 1000.0 / 9.80665);

			shell_print(sh, "accel x=%d y=%d z=%d mg", x, y, z);
		} else {
			shell_print(sh, "accel fetch err=%d", err);
		}
	} else {
		shell_print(sh, "accel not ready");
	}

	if (device_is_ready(sensor_shtc3)) {
		err = sensor_sample_fetch(sensor_shtc3);
		if (err == 0) {
			sensor_channel_get(sensor_shtc3, SENSOR_CHAN_AMBIENT_TEMP, &val[0]);
			sensor_channel_get(sensor_shtc3, SENSOR_CHAN_HUMIDITY, &val[1]);
			double temp = sensor_value_to_double(&val[0]);
			double hum = sensor_value_to_double(&val[1]);

			shell_print(sh, "temp=%.1f C  humidity=%.1f %%", temp, hum);
		} else {
			shell_print(sh, "shtc3 fetch err=%d", err);
		}
	} else {
		shell_print(sh, "shtc3 not ready");
	}

	int batt = read_battery_mv();

	shell_print(sh, "battery=%d mV", batt);
	return 0;
}

/* Shell command: toggle PSM */
static int cmd_psm(const struct shell *sh, size_t argc, char **argv)
{
	if (argc < 2) {
		shell_print(sh, "Usage: psm <on|off>");
		return -EINVAL;
	}

	if (strcmp(argv[1], "off") == 0 || strcmp(argv[1], "0") == 0) {
		int ret = bg9x_psm_set(false);

		app_psm_disabled = 1;
		settings_save_one("app/psm_disable", &app_psm_disabled, sizeof(app_psm_disabled));
		app_runtime_cfg_dirty = true;
		shell_print(sh, "PSM disabled (ret=%d) saved — persists across reboots", ret);
	} else if (strcmp(argv[1], "on") == 0 || strcmp(argv[1], "1") == 0) {
		int ret = bg9x_psm_set(true);

		app_psm_disabled = 0;
		settings_save_one("app/psm_disable", &app_psm_disabled, sizeof(app_psm_disabled));
		app_runtime_cfg_dirty = true;
		shell_print(sh, "PSM enabled (ret=%d) saved — persists across reboots", ret);
	} else {
		shell_print(sh, "Usage: psm <on|off>");
		return -EINVAL;
	}
	return 0;
}

/* Shell command: AT query (appends ? automatically since shell eats ?) */
static int cmd_atq(const struct shell *sh, size_t argc, char **argv)
{
	char cmd[128];
	char resp[512];
	size_t offset = 0;
	int ret;

	if (argc < 2) {
		shell_print(sh, "Usage: atq <AT command without ?>  (e.g. atq AT+CCLK)");
		return -EINVAL;
	}

	for (int i = 1; i < (int)argc && offset < sizeof(cmd) - 2; i++) {
		if (i > 1 && offset < sizeof(cmd) - 2) {
			cmd[offset++] = ' ';
		}
		size_t len = strlen(argv[i]);

		if (offset + len >= sizeof(cmd) - 1) {
			break;
		}
		memcpy(cmd + offset, argv[i], len);
		offset += len;
	}
	cmd[offset++] = '?';
	cmd[offset] = '\0';

	ret = bg9x_at_cmd_passthrough(cmd, resp, sizeof(resp));
	if (resp[0] != '\0') {
		shell_print(sh, "%s", resp);
	}
	shell_print(sh, "%s (ret=%d)", ret == 0 ? "OK" : "ERROR", ret);
	return 0;
}

/* Shell command: toggle GPS in cycle */
static int cmd_gps(const struct shell *sh, size_t argc, char **argv)
{
	if (argc < 2) {
		shell_print(sh, "GPS is %s", app_gps_enabled ? "ON" : "OFF");
		return 0;
	}

	if (strcmp(argv[1], "on") == 0 || strcmp(argv[1], "1") == 0) {
		app_gps_enabled = 1;
		settings_save_one("app/gps_enable", &app_gps_enabled, sizeof(app_gps_enabled));
		app_runtime_cfg_dirty = true;
		shell_print(sh, "GPS enabled — will attempt fix each cycle");
	} else if (strcmp(argv[1], "off") == 0 || strcmp(argv[1], "0") == 0) {
		app_gps_enabled = 0;
		settings_save_one("app/gps_enable", &app_gps_enabled, sizeof(app_gps_enabled));
		app_runtime_cfg_dirty = true;
		shell_print(sh, "GPS disabled — skipping fix in cycle");
	} else {
		shell_print(sh, "Usage: gps <on|off>");
		return -EINVAL;
	}
	return 0;
}

static int cmd_mode(const struct shell *sh, size_t argc, char **argv)
{
	if (argc < 2) {
		shell_print(sh, "mode=%s motion=%d tracking_motion=%u trigger=%d slow=%d",
			    app_tracker_mode_name(app_tracker_mode),
			    app_motion_detected ? 1 : 0,
			    app_tracking_motion_events,
			    app_motion_trigger_ready ? 1 : 0,
			    app_consecutive_slow_fixes);
		return 0;
	}

	if (strcmp(argv[1], "parked") == 0 || strcmp(argv[1], "p") == 0) {
		app_tracker_set_mode(TRACKER_PARKED, "shell");
		shell_print(sh, "mode set to PARKED");
		return 0;
	}

	if (strcmp(argv[1], "tracking") == 0 || strcmp(argv[1], "t") == 0) {
		app_tracker_set_mode(TRACKER_TRACKING, "shell");
		shell_print(sh, "mode set to TRACKING");
		return 0;
	}

	if (strcmp(argv[1], "motion") == 0) {
		app_motion_detected = true;
		shell_print(sh, "motion flag injected");
		return 0;
	}

	shell_print(sh, "Usage: mode [parked|tracking|motion]");
	return -EINVAL;
}

static int cmd_cycle(const struct shell *sh, size_t argc, char **argv)
{
	if (argc != 2 || strcmp(argv[1], "now") != 0) {
		shell_print(sh, "Usage: cycle now");
		return -EINVAL;
	}

	app_force_cycle_requested = true;
	LOG_INF("manual cycle requested via shell");
	shell_print(sh, "cycle request queued");
	return 0;
}

static int cmd_runcycle(const struct shell *sh, size_t argc, char **argv)
{
	if (argc != 1) {
		shell_error(sh, "usage: runcycle");
		return -EINVAL;
	}

	app_force_cycle_no_psm_requested = false;
	app_force_cycle_requested = true;
	LOG_INF("manual cycle requested via shell");
	shell_print(sh, "runcycle queued");
	return 0;
}

static int cmd_runcycle_nopsm(const struct shell *sh, size_t argc, char **argv)
{
	if (argc != 1) {
		shell_error(sh, "usage: runcycle_nopsm");
		return -EINVAL;
	}

	app_force_cycle_no_psm_requested = true;
	app_force_cycle_requested = true;
	LOG_INF("manual no-psm cycle requested via shell");
	shell_print(sh, "runcycle_nopsm queued");
	return 0;
}

static int cmd_cyclelog(const struct shell *sh, size_t argc, char **argv)
{
	if (argc == 1) {
		shell_print(sh, "cyclelog %s", app_cloud_cycle_log_enabled ? "on" : "off");
		return 0;
	}

	if (argc != 2) {
		shell_error(sh, "usage: cyclelog [on|off]");
		return -EINVAL;
	}

	if (strcmp(argv[1], "on") == 0) {
		app_cloud_cycle_log_enabled = true;
		shell_print(sh, "cyclelog on");
		return 0;
	}

	if (strcmp(argv[1], "off") == 0) {
		app_cloud_cycle_log_enabled = false;
		shell_print(sh, "cyclelog off");
		return 0;
	}

	shell_error(sh, "usage: cyclelog [on|off]");
	return -EINVAL;
}

static int cmd_settingssync(const struct shell *sh, size_t argc, char **argv)
{
	if (argc == 1) {
		shell_print(sh, "settingssync %s", app_settings_sync_enabled ? "on" : "off");
		return 0;
	}

	if (argc != 2) {
		shell_error(sh, "usage: settingssync [on|off]");
		return -EINVAL;
	}

	if (strcmp(argv[1], "on") == 0) {
		app_settings_sync_enabled = true;
		shell_print(sh, "settingssync on");
		return 0;
	}

	if (strcmp(argv[1], "off") == 0) {
		app_settings_sync_enabled = false;
		shell_print(sh, "settingssync off");
		return 0;
	}

	shell_error(sh, "usage: settingssync [on|off]");
	return -EINVAL;
}

static int cmd_mailboxpoll(const struct shell *sh, size_t argc, char **argv)
{
	if (argc == 1) {
		shell_print(sh, "mailboxpoll %s", app_mailbox_poll_enabled ? "on" : "off");
		return 0;
	}

	if (argc != 2) {
		shell_error(sh, "usage: mailboxpoll [on|off]");
		return -EINVAL;
	}

	if (strcmp(argv[1], "on") == 0) {
		app_mailbox_poll_enabled = true;
		shell_print(sh, "mailboxpoll on");
		return 0;
	}

	if (strcmp(argv[1], "off") == 0) {
		app_mailbox_poll_enabled = false;
		shell_print(sh, "mailboxpoll off");
		return 0;
	}

	shell_error(sh, "usage: mailboxpoll [on|off]");
	return -EINVAL;
}

static int cmd_led(const struct shell *sh, size_t argc, char **argv)
{
#if !defined(CONFIG_APP_LED_FEEDBACK)
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	shell_print(sh, "LED feedback is disabled at build time");
	return -ENOTSUP;
#else
	enum app_led_pattern pattern = APP_LED_OFF;

	if (argc < 2) {
		shell_print(sh,
			    "Usage: led <off|boot|gpson|gps|breathe|breathe50|heartbeat|payload|fix|motion|tracking-motion|connected|charging|error|auto>");
		return -EINVAL;
	}

	if (strcmp(argv[1], "auto") == 0) {
		app_led_release_override();
		shell_print(sh, "green LED pattern=auto");
		return 0;
	}

	if (strcmp(argv[1], "off") == 0) {
		pattern = APP_LED_OFF;
	} else if (strcmp(argv[1], "boot") == 0) {
		pattern = APP_LED_BOOT;
	} else if (strcmp(argv[1], "gpson") == 0) {
		pattern = APP_LED_GPS_POWER_ON;
	} else if (strcmp(argv[1], "gps") == 0 || strcmp(argv[1], "acquire") == 0) {
		pattern = APP_LED_GPS_ACQUIRING;
	} else if (strcmp(argv[1], "breathe") == 0) {
		pattern = APP_LED_BREATHE;
	} else if (strcmp(argv[1], "breathe50") == 0) {
		pattern = APP_LED_BREATHE50;
	} else if (strcmp(argv[1], "heartbeat") == 0) {
		pattern = APP_LED_HEARTBEAT;
	} else if (strcmp(argv[1], "fix") == 0) {
		pattern = APP_LED_GPS_FIX;
	} else if (strcmp(argv[1], "motion") == 0) {
		pattern = APP_LED_MOTION_WAKE;
	} else if (strcmp(argv[1], "tracking-motion") == 0) {
		pattern = APP_LED_TRACKING_MOTION;
	} else if (strcmp(argv[1], "connected") == 0) {
		pattern = APP_LED_CONNECTED;
	} else if (strcmp(argv[1], "payload") == 0) {
		pattern = APP_LED_PAYLOAD_SENT;
	} else if (strcmp(argv[1], "charging") == 0) {
		pattern = APP_LED_CHARGING;
	} else if (strcmp(argv[1], "error") == 0) {
		pattern = APP_LED_ERROR;
	} else {
		shell_print(sh,
			    "Usage: led <off|boot|gpson|gps|breathe|breathe50|heartbeat|payload|fix|motion|tracking-motion|connected|charging|error|auto>");
		return -EINVAL;
	}

	app_led_force_pattern(pattern);
	shell_print(sh, "green LED pattern=%s", argv[1]);
	return 0;
#endif
}

static int cmd_ledpct(const struct shell *sh, size_t argc, char **argv)
{
#if !defined(CONFIG_APP_LED_FEEDBACK)
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	shell_print(sh, "LED feedback is disabled at build time");
	return -ENOTSUP;
#else
	char *endptr;
	unsigned long pct;

	if (argc != 2) {
		shell_print(sh, "Usage: ledpct <0-100>");
		return -EINVAL;
	}

	pct = strtoul(argv[1], &endptr, 10);
	if ((endptr == argv[1]) || (*endptr != '\0') || (pct > 100UL)) {
		shell_print(sh, "Usage: ledpct <0-100>");
		return -EINVAL;
	}

	app_led_manual_brightness((uint8_t)pct);
	shell_print(sh, "green LED brightness=%lu%%", pct);
	return 0;
#endif
}

static bool app_tracker_mode_from_string(const char *value, enum tracker_mode *mode)
{
	if (value == NULL || mode == NULL) {
		return false;
	}

	if (strcmp(value, "TRACKING") == 0 || strcmp(value, "tracking") == 0) {
		*mode = TRACKER_TRACKING;
		return true;
	}

	if (strcmp(value, "PARKED") == 0 || strcmp(value, "parked") == 0) {
		*mode = TRACKER_PARKED;
		return true;
	}

	return false;
}

static bool app_led_pattern_from_string(const char *value, enum app_led_pattern *pattern)
{
	if (value == NULL || pattern == NULL) {
		return false;
	}

	if (strcmp(value, "off") == 0) {
		*pattern = APP_LED_OFF;
	} else if (strcmp(value, "boot") == 0) {
		*pattern = APP_LED_BOOT;
	} else if (strcmp(value, "gpson") == 0) {
		*pattern = APP_LED_GPS_POWER_ON;
	} else if (strcmp(value, "gps") == 0 || strcmp(value, "acquire") == 0) {
		*pattern = APP_LED_GPS_ACQUIRING;
	} else if (strcmp(value, "breathe") == 0) {
		*pattern = APP_LED_BREATHE;
	} else if (strcmp(value, "breathe50") == 0) {
		*pattern = APP_LED_BREATHE50;
	} else if (strcmp(value, "heartbeat") == 0) {
		*pattern = APP_LED_HEARTBEAT;
	} else if (strcmp(value, "fix") == 0) {
		*pattern = APP_LED_GPS_FIX;
	} else if (strcmp(value, "motion") == 0) {
		*pattern = APP_LED_MOTION_WAKE;
	} else if (strcmp(value, "tracking-motion") == 0) {
		*pattern = APP_LED_TRACKING_MOTION;
	} else if (strcmp(value, "connected") == 0) {
		*pattern = APP_LED_CONNECTED;
	} else if (strcmp(value, "payload") == 0) {
		*pattern = APP_LED_PAYLOAD_SENT;
	} else if (strcmp(value, "charging") == 0) {
		*pattern = APP_LED_CHARGING;
	} else if (strcmp(value, "error") == 0) {
		*pattern = APP_LED_ERROR;
	} else {
		return false;
	}

	return true;
}

static bool app_json_extract_string(const char *json, const char *key, char *out, size_t out_size)
{
	char pattern[40];
	const char *cursor;
	size_t out_len = 0U;
	int pattern_len;

	if (json == NULL || key == NULL || out == NULL || out_size == 0U) {
		return false;
	}

	pattern_len = snprintk(pattern, sizeof(pattern), "\"%s\"", key);
	if (pattern_len <= 0 || (size_t)pattern_len >= sizeof(pattern)) {
		return false;
	}

	cursor = strstr(json, pattern);
	if (cursor == NULL) {
		return false;
	}

	cursor = strchr(cursor + pattern_len, ':');
	if (cursor == NULL) {
		return false;
	}

	cursor++;
	while (*cursor != '\0' && isspace((unsigned char)*cursor)) {
		cursor++;
	}

	if (*cursor != '"') {
		return false;
	}

	cursor++;
	while (*cursor != '\0' && *cursor != '"') {
		if (*cursor == '\\' && cursor[1] != '\0') {
			cursor++;
		}

		if (out_len + 1U >= out_size) {
			return false;
		}

		out[out_len++] = *cursor++;
	}

	if (*cursor != '"') {
		return false;
	}

	out[out_len] = '\0';
	return true;
}

static bool app_json_extract_top_level_object(const char *json, const char *key,
					      char *out, size_t out_size)
{
	char pattern[40];
	const char *cursor;
	int pattern_len;
	size_t out_len = 0U;
	int depth = 0;

	if (json == NULL || key == NULL || out == NULL || out_size == 0U) {
		return false;
	}

	pattern_len = snprintk(pattern, sizeof(pattern), "\"%s\"", key);
	if (pattern_len <= 0 || (size_t)pattern_len >= sizeof(pattern)) {
		return false;
	}

	cursor = json;
	while ((cursor = strstr(cursor, pattern)) != NULL) {
		const char *value_start;
		const char *value_end;

		if (cursor > json && cursor[-1] == '\\') {
			cursor += pattern_len;
			continue;
		}

		value_start = strchr(cursor + pattern_len, ':');
		if (value_start == NULL) {
			return false;
		}

		value_start++;
		while (*value_start != '\0' && isspace((unsigned char)*value_start)) {
			value_start++;
		}

		if (*value_start != '{') {
			cursor = value_start;
			continue;
		}

		value_end = value_start;
		depth = 0;
		while (*value_end != '\0') {
			if (*value_end == '{') {
				depth++;
			} else if (*value_end == '}') {
				depth--;
				if (depth == 0) {
					value_end++;
					break;
				}
			}
			value_end++;
		}

		if (depth != 0) {
			return false;
		}

		out_len = (size_t)(value_end - value_start);
		if (out_len + 1U > out_size) {
			return false;
		}

		memcpy(out, value_start, out_len);
		out[out_len] = '\0';
		return true;
	}

	return false;
}

static size_t app_mailbox_collect_ids(const char *json,
				      struct app_mailbox_command_id *ids,
				      size_t max_ids)
{
	const char *cursor = json;
	size_t count = 0U;
	int depth = 0;

	if (json == NULL || ids == NULL || max_ids == 0U) {
		return 0U;
	}

	while (*cursor != '\0') {
		if (*cursor == '{') {
			depth++;
			cursor++;
			continue;
		}

		if (*cursor == '}') {
			depth--;
			cursor++;
			continue;
		}

		if (*cursor != '"' || depth != 1) {
			cursor++;
			continue;
		}

		cursor++;
		{
			const char *start = cursor;
			size_t key_len = 0U;

			while (cursor[key_len] != '\0' && cursor[key_len] != '"') {
				if (cursor[key_len] == '\\' && cursor[key_len + 1] != '\0') {
					key_len += 2U;
				} else {
					key_len++;
				}
			}

			if (cursor[key_len] != '"') {
				break;
			}

			cursor += key_len + 1U;
			while (*cursor != '\0' && isspace((unsigned char)*cursor)) {
				cursor++;
			}

			if (*cursor != ':') {
				continue;
			}

			if (count < max_ids && key_len < sizeof(ids[count].value)) {
				memcpy(ids[count].value, start, key_len);
				ids[count].value[key_len] = '\0';
				count++;
			}
		}
	}

	return count;
}

static void app_mailbox_sort_ids(struct app_mailbox_command_id *ids, size_t count)
{
	for (size_t i = 1; i < count; i++) {
		struct app_mailbox_command_id current = ids[i];
		size_t j = i;

		while (j > 0U && strcmp(ids[j - 1U].value, current.value) > 0) {
			ids[j] = ids[j - 1U];
			j--;
		}

		ids[j] = current;
	}
}

static bool app_mailbox_publish_result(const char *id, const char *type,
				       const char *status, const char *detail)
{
	char path[APP_STATE_PATH_MAX];
	char payload[APP_CMD_RESULT_MAX];
	int len;
	enum golioth_status gstatus;

	if (!app_path_join(path, sizeof(path), APP_CMD_RESULT_ROOT, id)) {
		return false;
	}

	len = snprintk(payload, sizeof(payload),
		       "{\"id\":\"%s\",\"type\":\"%s\",\"status\":\"%s\",\"detail\":\"%s\"}",
		       id, type, status, detail);
	if (len <= 0 || (size_t)len >= sizeof(payload)) {
		return false;
	}

	gstatus = golioth_lightdb_set_async(client, path, GOLIOTH_CONTENT_TYPE_JSON,
					    (const uint8_t *)payload, strlen(payload),
					    NULL, NULL);
	if (gstatus != GOLIOTH_OK) {
		LOG_WRN("mailbox result publish failed id=%s status=%s",
			id, golioth_status_to_str(gstatus));
		return false;
	}

	app_cycle_tx_bytes_add(app_lightdb_string_payload_bytes(path, strlen(payload)));

	return true;
}

static bool app_mailbox_delete_pending(const char *id)
{
	char path[APP_STATE_PATH_MAX];
	enum golioth_status status;

	if (!app_path_join(path, sizeof(path), APP_CMD_PENDING_ROOT, id)) {
		return false;
	}

	status = golioth_lightdb_delete_async(client, path, NULL, NULL);
	if (status != GOLIOTH_OK) {
		LOG_WRN("mailbox delete failed id=%s status=%s",
			id, golioth_status_to_str(status));
		return false;
	}

	return true;
}

static void app_mailbox_execute_command(const char *id, const char *json, int *batt_mv)
{
	char type[32] = "unknown";
	char arg[32];
	const char *result_status = "invalid";
	const char *result_detail = "missing-type";

	if (!app_json_extract_string(json, "type", type, sizeof(type))) {
		goto out;
	}

	if (strcmp(type, "set_mode") == 0) {
		enum tracker_mode mode;

		if (!app_json_extract_string(json, "mode", arg, sizeof(arg))) {
			result_detail = "missing-mode";
			goto out;
		}

		if (!app_tracker_mode_from_string(arg, &mode)) {
			result_detail = "bad-mode";
			goto out;
		}

		app_tracker_set_mode(mode, "mailbox");
		result_status = "ok";
		result_detail = app_tracker_mode_name(mode);
	} else if (strcmp(type, "led_pattern") == 0) {
		enum app_led_pattern pattern;

		if (!app_json_extract_string(json, "pattern", arg, sizeof(arg))) {
			result_detail = "missing-pattern";
			goto out;
		}

		if (!app_led_pattern_from_string(arg, &pattern)) {
			result_detail = "bad-pattern";
			goto out;
		}

		app_led_force_pattern(pattern);
		result_status = "ok";
		result_detail = arg;
	} else if (strcmp(type, "disable_sleep") == 0) {
		int ret = bg9x_psm_set(false);

		app_psm_disabled = 1U;
		(void)app_save_u32_setting("app/psm_disable", app_psm_disabled);
		app_runtime_cfg_dirty = true;
		result_status = (ret < 0) ? "error" : "ok";
		result_detail = (ret < 0) ? "psm-off-failed" : "sleep-disabled";
	} else if (strcmp(type, "resume_sleep") == 0) {
		int ret = bg9x_psm_set(true);

		app_psm_disabled = 0U;
		(void)app_save_u32_setting("app/psm_disable", app_psm_disabled);
		app_runtime_cfg_dirty = true;
		result_status = (ret < 0) ? "error" : "ok";
		result_detail = (ret < 0) ? "psm-on-failed" : "sleep-enabled";
	} else if (strcmp(type, "sample_sensors") == 0) {
		read_sensors();
		if (batt_mv != NULL) {
			*batt_mv = read_battery_mv();
		}
		result_status = "ok";
		result_detail = "sampled";
	} else if (strcmp(type, "check_for_update") == 0) {
		result_status = "unsupported";
		result_detail = "ota-not-implemented";
	} else if (strcmp(type, "reboot") == 0) {
		app_reboot_requested = true;
		result_status = "ok";
		result_detail = "scheduled";
	} else {
		result_status = "unsupported";
		result_detail = "unknown-command";
	}

out:
	if (app_mailbox_publish_result(id, type, result_status, result_detail)) {
		(void)app_mailbox_delete_pending(id);
	}

	LOG_INF("mailbox command id=%s type=%s result=%s detail=%s",
		id, type, result_status, result_detail);
}

static void app_mailbox_process(int *batt_mv)
{
	struct app_mailbox_command_id ids[APP_MAILBOX_MAX_COMMANDS];
	size_t payload_size = sizeof(app_mailbox_pending_json) - 1U;
	enum golioth_status status;
	size_t count;

	app_mailbox_pending_json_len = 0U;
	app_mailbox_pending_json[0] = '\0';

	status = golioth_lightdb_get_sync(client, APP_CMD_PENDING_ROOT,
					  GOLIOTH_CONTENT_TYPE_JSON,
					  (uint8_t *)app_mailbox_pending_json,
					  &payload_size, 1);
	if (status != GOLIOTH_OK) {
		LOG_DBG("mailbox pending read skipped status=%s",
			golioth_status_to_str(status));
		return;
	}

	app_mailbox_pending_json[payload_size] = '\0';
	app_mailbox_pending_json_len = payload_size;
	if (app_mailbox_pending_json_len == 0U ||
	    strcmp(app_mailbox_pending_json, "{}") == 0 ||
	    golioth_payload_is_null((const uint8_t *)app_mailbox_pending_json,
				    app_mailbox_pending_json_len)) {
		LOG_DBG("mailbox pending read returned empty payload");
		return;
	}

	count = app_mailbox_collect_ids(app_mailbox_pending_json, ids, ARRAY_SIZE(ids));
	if (count == 0U) {
		LOG_DBG("mailbox pending read returned no command ids");
		return;
	}

	app_mailbox_sort_ids(ids, count);
	LOG_INF("mailbox pending count=%u", (unsigned int)count);

	for (size_t i = 0; i < count; i++) {
		char command_json[APP_CMD_JSON_MAX];

		if (!app_json_extract_top_level_object(app_mailbox_pending_json, ids[i].value,
						       command_json, sizeof(command_json))) {
			LOG_WRN("mailbox object extract failed id=%s", ids[i].value);
			continue;
		}

		app_mailbox_execute_command(ids[i].value, command_json, batt_mv);
	}
}

SHELL_CMD_REGISTER(at, NULL, "Send AT command to BG95 modem", cmd_at);
SHELL_CMD_REGISTER(atq, NULL, "AT query (appends ? for you): atq AT+CCLK", cmd_atq);
SHELL_CMD_REGISTER(cell, NULL, "Read cell info from BG95", cmd_cell);
SHELL_CMD_REGISTER(qgdcnt, NULL, "Read BG95 modem data counters", cmd_qgdcnt);
SHELL_CMD_REGISTER(modemsnap, NULL, "Read BG95 runtime snapshot: modemsnap [wake]", cmd_modemsnap);
SHELL_CMD_REGISTER(uart1probe, NULL, "Write direct test line to UART1: uart1probe [tag]", cmd_uart1probe);
SHELL_CMD_REGISTER(connectonly, NULL, "Connect Golioth without a cycle: connectonly [timeout_s]", cmd_connectonly);
SHELL_CMD_REGISTER(resume_socket, NULL, "Prepare runtime wake and recreate the connected BG95 socket if needed", cmd_resume_socket);
SHELL_CMD_REGISTER(sendprobe, NULL, "Send tiny probe payload to debug path: sendprobe [1-8]", cmd_sendprobe);
SHELL_CMD_REGISTER(sendtracker, NULL, "Send cached tracker payload on active session: sendtracker", cmd_sendtracker);
SHELL_CMD_REGISTER(resumeprobe, NULL, "Idle while connected, then send probe: resumeprobe [idle_s] [1-8]", cmd_resumeprobe);
SHELL_CMD_REGISTER(resumeprobe_nopsm, NULL, "Idle while connected with PSM disabled, then send probe: resumeprobe_nopsm [idle_s] [1-8]", cmd_resumeprobe_nopsm);
SHELL_CMD_REGISTER(resumetracker, NULL, "Idle while connected, then send cached tracker payload: resumetracker [idle_s] [count]", cmd_resumetracker);
SHELL_CMD_REGISTER(disconnect, NULL, "Disconnect Golioth client", cmd_disconnect);
SHELL_CMD_REGISTER(ntime, NULL, "Read network time from BG95", cmd_ntime);
SHELL_CMD_REGISTER(sensors, NULL, "Read all onboard sensors", cmd_sensors);
SHELL_CMD_REGISTER(psm, NULL, "Toggle PSM: psm <on|off>", cmd_psm);
SHELL_CMD_REGISTER(gps, NULL, "Toggle GPS: gps <on|off>", cmd_gps);
SHELL_CMD_REGISTER(led, NULL, "Force green LED pattern: led <off|boot|gpson|gps|breathe|breathe50|heartbeat|payload|fix|motion|connected|charging|error|auto>", cmd_led);
SHELL_CMD_REGISTER(ledpct, NULL, "Set steady green LED brightness: ledpct <0-100>", cmd_ledpct);
SHELL_CMD_REGISTER(mode, NULL, "Tracker mode: mode [parked|tracking|motion]", cmd_mode);
SHELL_CMD_REGISTER(cycle, NULL, "Wake the app wait loop once: cycle now", cmd_cycle);
SHELL_CMD_REGISTER(runcycle, NULL, "Run one manual cycle and stay awake", cmd_runcycle);
SHELL_CMD_REGISTER(runcycle_nopsm, NULL, "Run one manual cycle without PSM re-enable", cmd_runcycle_nopsm);
SHELL_CMD_REGISTER(cyclelog, NULL, "Toggle cloud cycle log: cyclelog [on|off]", cmd_cyclelog);
SHELL_CMD_REGISTER(settingssync, NULL, "Toggle settings sync: settingssync [on|off]", cmd_settingssync);
SHELL_CMD_REGISTER(mailboxpoll, NULL, "Toggle mailbox poll: mailboxpoll [on|off]", cmd_mailboxpoll);

int main(void)
{
	int counter = 1;
	int consecutive_failures = 0;

	LOG_DBG("start tracker app");
	printk("rak5010 tracker start\n");
	app_uart1_write_line("UART1 PROBE tag=boot stage=main-entry");
	app_uart1_write_line("UART1 log-only mode");
	app_log_reset_cause();

	/* Configure GPS antenna power MOSFET (P1.07) as output, initially OFF */
	if (device_is_ready(gps_ant_gpio)) {
		gpio_pin_configure(gps_ant_gpio, GPS_ANT_PIN, GPIO_OUTPUT_INACTIVE);
		LOG_INF("GPS antenna power GPIO (P1.07) configured");
	} else {
		LOG_ERR("GPIO1 device not ready for GPS antenna power");
	}

	app_settings_init();
	(void)bg9x_gps_xtra_policy_set(app_xtra_enabled != 0U,
				       (uint8_t)app_xtra_retention_days,
				       (uint8_t)app_xtra_mode);

	if (app_psm_disabled == 1) {
		LOG_INF("PSM override: disabled by saved setting");
		bg9x_psm_set(false);
	}

	app_wdt_init();

	if (app_motion_trigger_init() < 0) {
		LOG_WRN("motion trigger init failed; forcing TRACKING mode");
		app_tracker_set_mode(TRACKER_TRACKING, "motion-init-failed");
	}

#if defined(CONFIG_MODEM_CELLULAR)
	modem_diag_init();
	printk("waiting for AT diag pipe\n");

	if (modem_diag_wait_ready(MODEM_DIAG_READY_TIMEOUT_SECONDS)) {
		printk("AT diag pipe ready\n");
		modem_diag_snapshot("boot-pre-net");
		modem_diag_configure_psm();
	} else {
		printk("AT diag pipe timeout\n");
		LOG_WRN("modem diagnostic pipe not ready after %ds",
			MODEM_DIAG_READY_TIMEOUT_SECONDS);
	}
#endif

	printk("waiting for network via net_connect()\n");
	LOG_INF("waiting for network via net_connect()");
	net_connect();
	printk("net_connect returned\n");
	LOG_INF("net_connect() returned");
	log_iface_state("post-net-connect");

	if (modem_diag_ready()) {
		modem_diag_snapshot("post-net-connect");
	}

	app_led_init();

	app_client_config = golioth_sample_credentials_get();
	/* NOTE: client created AFTER GPS (golioth_client_create auto-starts
	 * the CoAP thread, which would make DNS/socket calls that conflict
	 * with GPS's CFUN=0 state). Created lazily on first connect. */

	while (true) {
		if (app_manual_cycle_mode_enabled()) {
			app_wait_manual_cycle_request("main-idle");
			app_force_cycle_requested = false;
			app_cycle_no_psm_active = app_force_cycle_no_psm_requested;
			app_force_cycle_no_psm_requested = false;
			LOG_INF("manual cycle request consumed before cycle start%s",
				app_cycle_no_psm_active ? " (nopsm)" : "");
		}

		if (app_tracker_mode == TRACKER_PARKED && app_motion_detected) {
			app_motion_detected = false;
			app_tracker_set_mode(TRACKER_TRACKING, "motion-pending");
		}

		bool parked_mode = (app_tracker_mode == TRACKER_PARKED);
		bool heartbeat_only = false;
		uint32_t next_sleep_seconds;
		uint32_t next_wake_seconds;
		int64_t cycle_anchor_ms;
		struct bg9x_gps_fix gps_fix = {0};
		struct bg9x_cell_info cell_info = {0};
		struct bg9x_network_time network_time = {0};
		int gps_ret;

		app_wdt_feed();
		app_cycle_counter_advance();
		app_cycle_tx_bytes_reset();
		app_tracking_motion_events_last_interval = 0U;
		cycle_anchor_ms = k_uptime_get();
		LOG_INF("cycle start cy=%d mode=%s fails=%d bt=%u tot=%u",
			counter, app_tracker_mode_name(app_tracker_mode), consecutive_failures,
			app_boot_count, app_total_cycles);
		log_iface_state("cycle-start");

		gps_ret = bg9x_cycle_begin();
		if (gps_ret < 0) {
			LOG_WRN("cycle begin wake/AT-ready prep failed: %d", gps_ret);
		}

		if (counter > 1 && modem_diag_ready()) {
			modem_diag_snapshot("post-sleep");
		}

		/* --- GPS acquisition (kills cellular temporarily) --- */
		if (app_gps_enabled) {
			app_wdt_feed();
			/* Power on GPS antenna MOSFET (P1.07 HIGH) */
			gpio_pin_set(gps_ant_gpio, GPS_ANT_PIN, 1);
			LOG_INF("#%d GPS antenna power ON (P1.07)", counter);
#if defined(CONFIG_APP_LED_GPS_FIX)
			app_led_pattern(APP_LED_GPS_POWER_ON);
			k_sleep(K_MSEC(160));
#endif
			k_sleep(K_MSEC(100)); /* settle time */

			gps_ret = bg9x_gps_session_begin();
			if (gps_ret == 0) {
				LOG_INF("#%d GPS session started, polling for fix (%us)...", counter,
					app_gps_timeout_seconds);
#if defined(CONFIG_APP_LED_GPS_FIX)
				app_led_pattern(APP_LED_GPS_ACQUIRING);
#endif
				gps_ret = bg9x_gps_get_fix(&gps_fix,
							  (int)app_gps_timeout_seconds);
				bg9x_gps_session_end();
				app_wdt_feed();

				if (gps_ret == 0 && gps_fix.valid) {
					LOG_INF("#%d GPS fix: lat=%.6f lon=%.6f alt=%.1f spd=%.1f sats=%u fix=%ums",
						counter, gps_fix.latitude, gps_fix.longitude,
						(double)gps_fix.altitude, (double)gps_fix.speed_kmh,
						gps_fix.sats_in_use, gps_fix.fix_time_ms);
					app_gps_fix_store(&gps_fix);
#if defined(CONFIG_APP_LED_GPS_FIX)
					app_led_pattern(APP_LED_GPS_FIX);
#endif
				} else {
					LOG_WRN("#%d GPS fix failed (ret=%d valid=%d)", counter,
						gps_ret, gps_fix.valid);
#if defined(CONFIG_APP_LED_GPS_FIX)
					app_led_pattern(APP_LED_OFF);
#endif
				}
			} else {
				LOG_WRN("#%d GPS session begin failed: %d", counter, gps_ret);
			}
			/* Power off GPS antenna MOSFET (P1.07 LOW) */
			gpio_pin_set(gps_ant_gpio, GPS_ANT_PIN, 0);
			LOG_INF("#%d GPS antenna power OFF", counter);
		} else {
			LOG_INF("#%d GPS disabled, skipping", counter);
		}

		/* --- Read sensors (accel, temp/humidity, battery) --- */
		read_sensors();
		int batt = read_battery_mv();

		/* --- Cell info + network time (cellular is restored) --- */
		bg9x_cell_info_get(&cell_info);
		if (cell_info.valid &&
		    cell_info.rat[0] != '\0' &&
		    cell_info.mcc != 0U &&
		    cell_info.mnc != 0U &&
		    cell_info.cell_id != 0U &&
		    cell_info.earfcn != 0U &&
		    cell_info.band != 0U &&
		    !(cell_info.rsrp == 0 &&
		      cell_info.rsrq == 0 &&
		      cell_info.sinr == 0)) {
			app_last_valid_cell_info = cell_info;
			app_last_valid_cell_info_present = true;
			LOG_INF("#%d cell: RAT=%s MCC=%u MNC=%u cellID=%x RSRP=%d SINR=%d",
				counter, cell_info.rat, cell_info.mcc, cell_info.mnc,
				cell_info.cell_id, cell_info.rsrp, cell_info.sinr);
		} else if (app_last_valid_cell_info_present) {
			cell_info = app_last_valid_cell_info;
			LOG_DBG("#%d cell: ignoring incomplete modem snapshot, keeping cellID=%x",
				counter, cell_info.cell_id);
		}
		(void)bg9x_network_time_get(&network_time);
		if (network_time.valid) {
			LOG_INF("#%d net_time: %04u-%02u-%02uT%02u:%02u:%02u tz=%d",
				counter, network_time.year, network_time.month,
				network_time.day, network_time.hour, network_time.min,
				network_time.sec, network_time.tz_quarter_hours / 4);
		}

		/* --- Create Golioth client lazily (after GPS, cellular restored) --- */
		bool client_created_this_cycle = false;
		if (client == NULL) {
			client = golioth_client_create(app_client_config);
			golioth_debug_set_cloud_log_enabled(false);
			golioth_client_register_event_callback(client, on_client_event, NULL);
			client_created_this_cycle = true;
			LOG_INF("Golioth client created");
		}

		app_modem_usage_cycle_snapshot(counter, "pre-connect", true);
		if (!client_created_this_cycle && golioth_client_is_running(client)) {
			LOG_WRN("#%d client thread running before connect; forcing clean reconnect", counter);
			golioth_client_stop(client);
		}

		k_sem_reset(&connected);
		golioth_client_start(client);
		LOG_INF("#%d waiting %us for connect", counter, app_cellular_timeout_seconds);
		app_wdt_feed();

		if (k_sem_take(&connected, K_SECONDS(app_cellular_timeout_seconds)) != 0) {
			LOG_WRN("#%d CONNECT TIMEOUT", counter);
			log_iface_state("connect-timeout");
			if (modem_diag_ready()) {
				modem_diag_snapshot("connect-timeout");
			}
#if defined(CONFIG_APP_STATUS_MARKERS)
			LOG_INF("status marker local-only value=E reason=connect-timeout");
#endif
			golioth_client_stop(client);
			consecutive_failures++;
			counter++;
			next_sleep_seconds = app_failure_sleep_seconds(consecutive_failures);
			if (parked_mode) {
				next_sleep_seconds = MAX(next_sleep_seconds,
							 app_parked_interval_seconds);
			}
			LOG_WRN("connect failure count=%d mode=%s sleeping %us before retry",
				consecutive_failures, app_tracker_mode_name(app_tracker_mode),
				next_sleep_seconds);
			app_sleep_after_failure(client, counter, next_sleep_seconds,
					      next_sleep_seconds, "CONNECT TIMEOUT");
			continue;
		}

		LOG_INF("#%d connected!", counter);
		consecutive_failures = 0;
		log_iface_state("connected");

		/*
		 * Let the newly established CoAP session settle before the first tracker
		 * publish. This is intentionally longer than the old 3s quiet window
		 * because the first stream send has been the most failure-prone part of
		 * the active cycle on BG95 resume.
		 */
		app_wait_for_golioth_idle("post-connect", false);
		app_wdt_feed();

		if (client_created_this_cycle) {
			LOG_INF("First client session uses the initial connected session; warmup reconnect disabled");
		}

		app_refresh_connected_network_snapshot(counter, &cell_info, &network_time);
		app_modem_usage_cycle_snapshot(counter, "post-connect", false);

#if defined(CONFIG_APP_STATUS_MARKERS)
		{
			char status_code = app_status_marker_connect_code(heartbeat_only);

			publish_status_marker(status_code, "post-connect");
			if (status_code == 'W') {
				app_pending_status_marker = '\0';
			}
		}
#endif

		/* --- Publish all telemetry --- */
		app_bytes_today_check_rollover();
		app_tracker_note_fix(&gps_fix);
		next_wake_seconds = app_next_wake_seconds(heartbeat_only);
		if (app_wake_anchored_intervals != 0U) {
			int64_t elapsed_ms = k_uptime_get() - cycle_anchor_ms;
			int64_t target_ms = (int64_t)next_wake_seconds * MSEC_PER_SEC;

			if (elapsed_ms >= target_ms) {
				next_sleep_seconds = 0U;
			} else {
				next_sleep_seconds =
					(uint32_t)DIV_ROUND_UP((uint32_t)(target_ms - elapsed_ms),
								   MSEC_PER_SEC);
			}
		} else {
			next_sleep_seconds = next_wake_seconds;
		}

		if (!tracker_pipeline_state_enabled()) {
			if (batt >= 0) {
				publish_battery_state_mv(batt);
			}
			if (last_batt_pct >= 0) {
				publish_battery_state_pct(last_batt_pct);
			}

			publish_tracker_mode_state();
			publish_motion_interrupts_state();
			publish_gps_state(&gps_fix);
			publish_environment_state();
			publish_cell_state(&cell_info);
			publish_network_time_state(&network_time);
			publish_modem_wake_stats();
		}
		{
			enum tracker_publish_result tracker_result;
			bool skip_cycle_log = false;

			tracker_result = publish_tracker_stream_record(counter, heartbeat_only,
								       batt, next_wake_seconds,
								       &gps_fix, &cell_info,
								       &network_time);
			app_modem_usage_cycle_snapshot(counter, "post-tracker", false);
			if (tracker_result == TRACKER_PUBLISH_TIMEOUT) {
				LOG_WRN("tracker publish timed out after payload send; skipping cycle log on degraded session");
				skip_cycle_log = true;
			} else if (tracker_result != TRACKER_PUBLISH_SENT) {
				LOG_WRN("tracker publish failed; suppressing reconnect retry to avoid duplicate uploads");
			}
			if (!skip_cycle_log) {
				uint32_t cycle_log_tx_bytes = 0U;
				bool cycle_log_sent = send_cycle_log(counter, heartbeat_only,
								     batt, next_wake_seconds,
								     &cycle_log_tx_bytes);

#if defined(CONFIG_APP_LED_FEEDBACK)
				if (cycle_log_sent) {
					app_led_pattern(APP_LED_PAYLOAD_SENT);
				}
#endif
				ARG_UNUSED(cycle_log_tx_bytes);
			} else {
				LOG_INF("cycle log skipped after tracker publish timeout");
			}

			app_modem_usage_cycle_snapshot(counter, "post-payload", false);

			if (tracker_result == TRACKER_PUBLISH_SENT &&
			    golioth_client_is_connected(client) &&
			    (app_settings_sync_enabled || app_mailbox_poll_enabled)) {
				app_wait_for_golioth_idle("post-telemetry", false);
				app_wdt_feed();
				if (app_settings_sync_enabled) {
					app_golioth_settings_register();
					app_wait_for_golioth_idle("post-telemetry-settings", true);
					app_wdt_feed();
				} else {
					LOG_INF("settings sync skipped (runtime disabled)");
				}
				if (app_mailbox_poll_enabled) {
					prune_legacy_command_state_once();
					app_mailbox_process(&batt);
				} else {
					LOG_INF("mailbox poll skipped (runtime disabled)");
				}
				if (app_settings_sync_enabled) {
					app_golioth_settings_unregister();
				}
			}
		}

		if (app_settings_sync_enabled) {
			app_golioth_settings_unregister();
		}

		/* --- Disconnect and sleep --- */
		if (app_reboot_requested) {
			LOG_WRN("mailbox requested reboot; restarting now");
			golioth_client_stop(client);
			k_sleep(K_MSEC(250));
			sys_reboot(SYS_REBOOT_COLD);
		}

#if defined(CONFIG_APP_STATUS_MARKERS)
		if (app_pending_status_marker == 'P') {
			publish_status_marker('P', "tracking-to-parked");
			app_pending_status_marker = '\0';
		}

		publish_status_marker('S', "pre-sleep");
		if (app_status_marker_cloud_enabled()) {
			k_sleep(K_MSEC(500));
		}
#endif
		app_wdt_feed();
		k_sleep(K_SECONDS(5));
		app_modem_usage_cycle_snapshot(counter, "pre-sleep-final", false);
		golioth_client_stop(client);
		if (!app_cycle_no_psm_active && app_psm_disabled == 0U) {
			int psm_ret = bg9x_psm_set_interval(true, next_wake_seconds);

			if (psm_ret < 0) {
				LOG_WRN("pre-sleep PSM re-enable failed: %d", psm_ret);
			}
		}
		app_led_pattern(APP_LED_OFF);
		if (app_manual_cycle_mode_enabled()) {
			LOG_INF("#%d debug cycle complete; staying awake for next shell command", counter);
			app_cycle_no_psm_active = false;
			counter++;
			continue;
		}
		if (app_tracker_mode == TRACKER_TRACKING) {
			LOG_INF("#%d stopped, tracking sleep %us (target=%us anchored=%u)",
				counter, next_sleep_seconds, next_wake_seconds,
				app_wake_anchored_intervals);
			counter++;
			app_wdt_feed();
			app_wait_tracking_interval(next_sleep_seconds);
		} else {
			LOG_INF("#%d stopped, parked sleep %us (target=%us anchored=%u)",
				counter, next_sleep_seconds, next_wake_seconds,
				app_wake_anchored_intervals);
			counter++;
			app_wdt_feed();
			app_wait_parked_interval(next_sleep_seconds);
		}
	}

	return 0;
}
