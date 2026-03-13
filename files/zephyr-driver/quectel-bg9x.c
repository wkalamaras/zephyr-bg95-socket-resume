/*
 * Copyright (c) 2020 Analog Life LLC
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT quectel_bg9x

#include <zephyr/posix/fcntl.h>
#include <zephyr/drivers/modem/quectel_bg9x.h>
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(modem_quectel_bg9x, CONFIG_MODEM_LOG_LEVEL);

#include "quectel-bg9x.h"

static struct k_thread	       modem_rx_thread;
static struct k_work_q	       modem_workq;
static struct modem_data       mdata;
static struct modem_context    mctx;
static const struct socket_op_vtable offload_socket_fd_op_vtable;
static int offload_socket_set_flags(const struct modem_socket *sock, int flags);
static int offload_connect(void *obj, const struct sockaddr *addr, socklen_t addrlen);
static bool modem_runtime_line_asserted(const struct gpio_dt_spec *gpio);
static int modem_optional_gpio_get(const struct gpio_dt_spec *gpio);
static int modem_ensure_ready(void);
static void modem_ap_ready_set(bool host_awake, const char *reason);
static int modem_runtime_at_ping(k_timeout_t timeout);
static void modem_log_runtime_lines_info(const char *label);
int bg9x_cycle_begin(void);
enum bg9x_qlts_state {
	BG9X_QLTS_STATE_UNKNOWN = 0,
	BG9X_QLTS_STATE_SUPPORTED,
	BG9X_QLTS_STATE_UNAVAILABLE,
};
static uint32_t runtime_native_wake_count;
static uint32_t runtime_emergency_reinit_count;
static uint32_t runtime_utc_day_value;
static int runtime_utc_day_result;
static enum bg9x_qlts_state runtime_qlts_state = BG9X_QLTS_STATE_UNKNOWN;

static int bg9x_parse_data_usage(const char *resp, struct bg9x_data_usage *usage)
{
	const char *cursor;
	unsigned long long tx_bytes;
	unsigned long long rx_bytes;

	if (resp == NULL || usage == NULL) {
		return -EINVAL;
	}

	cursor = strstr(resp, "+QGDCNT:");
	if (cursor == NULL) {
		return -ENOENT;
	}

	cursor += strlen("+QGDCNT:");
	while (*cursor == ' ' || *cursor == '\t') {
		cursor++;
	}

	if (sscanf(cursor, "%llu,%llu", &tx_bytes, &rx_bytes) != 2) {
		return -EINVAL;
	}

	usage->valid = true;
	usage->tx_bytes = (uint64_t)tx_bytes;
	usage->rx_bytes = (uint64_t)rx_bytes;
	return 0;
}

/* GPS fix working state — filled by response handlers */
static struct bg9x_gps_fix gps_pending_fix;
static bool gps_fix_received;
static char gps_nmea_buf[256];

/* Cell info working state */
static struct bg9x_cell_info cell_pending_info;
static bool cell_info_received;

int bg9x_runtime_stats_get(struct bg9x_runtime_stats *stats)
{
	if (stats == NULL) {
		return -EINVAL;
	}

	stats->native_wake_count = runtime_native_wake_count;
	stats->emergency_reinit_count = runtime_emergency_reinit_count;

	return 0;
}

int bg9x_data_usage_get(struct bg9x_data_usage *usage)
{
	char resp[96];
	int ret;

	if (usage == NULL) {
		return -EINVAL;
	}

	memset(usage, 0, sizeof(*usage));
	ret = bg9x_at_cmd_passthrough("AT+QGDCNT?", resp, sizeof(resp));
	if (ret < 0) {
		return ret;
	}

	return bg9x_parse_data_usage(resp, usage);
}

static int bg9x_days_from_civil(int year, unsigned int month, unsigned int day)
{
	year -= month <= 2U;
	const int era = (year >= 0 ? year : year - 399) / 400;
	const unsigned int yoe = (unsigned int)(year - era * 400);
	const unsigned int doy = (153U * (month + (month > 2U ? (unsigned int)-3 : 9U)) + 2U) / 5U +
				 day - 1U;
	const unsigned int doe = yoe * 365U + yoe / 4U - yoe / 100U + doy;

	return era * 146097 + (int)doe - 719468;
}

static int bg9x_parse_utc_day(const char *cclk_value, uint32_t *utc_day)
{
	char buf[40];
	size_t out = 0U;
	int year;
	int month;
	int day;
	int hour;
	int minute;
	int second;
	int offset_quarters;
	int dst;
	int offset_minutes;
	int local_minutes;
	int utc_minutes;
	int day_adjust = 0;
	char sign;
	int matched;

	if (cclk_value == NULL || utc_day == NULL) {
		return -EINVAL;
	}

	for (size_t i = 0U; cclk_value[i] != '\0' && out < (sizeof(buf) - 1U); i++) {
		if (cclk_value[i] == '"') {
			continue;
		}

		buf[out++] = cclk_value[i];
	}
	buf[out] = '\0';

	dst = 0;
	matched = sscanf(buf, "%d/%d/%d,%d:%d:%d%c%d,%d",
			 &year, &month, &day, &hour, &minute, &second, &sign,
			 &offset_quarters, &dst);
	if (matched != 9) {
		matched = sscanf(buf, "%d/%d/%d,%d:%d:%d%c%d",
				 &year, &month, &day, &hour, &minute, &second, &sign,
				 &offset_quarters);
	}
	if (matched != 8 && matched != 9) {
		return -EINVAL;
	}

	if (sign != '+' && sign != '-') {
		return -EINVAL;
	}

	if (year < 100) {
		year += 2000;
	}
	offset_minutes = offset_quarters * 15;
	if (sign == '-') {
		offset_minutes = -offset_minutes;
	}

	local_minutes = (hour * 60) + minute;
	utc_minutes = local_minutes - offset_minutes;
	while (utc_minutes < 0) {
		utc_minutes += 24 * 60;
		day_adjust--;
	}
	while (utc_minutes >= 24 * 60) {
		utc_minutes -= 24 * 60;
		day_adjust++;
	}

	*utc_day = (uint32_t)(bg9x_days_from_civil(year, (unsigned int)month,
					 (unsigned int)day) + day_adjust);
	return 0;
}

static int bg9x_parse_time_response_argv(uint16_t argc, uint8_t **argv, uint32_t *utc_day,
					 char *buf, size_t buf_size)
{
	size_t offset = 0U;

	if (argc < 1U || utc_day == NULL || buf == NULL || buf_size == 0U) {
		return -EINVAL;
	}

	buf[0] = '\0';
	for (uint16_t i = 0U; i < argc; i++) {
		offset += snprintk(buf + offset, buf_size - offset,
				   i == 0U ? "%s" : ",%s", (char *)argv[i]);
		if (offset >= buf_size) {
			return -ENOMEM;
		}
	}

	return bg9x_parse_utc_day(buf, utc_day);
}

MODEM_CMD_DEFINE(on_cmd_atcmdinfo_cclk)
{
	char cclk_buf[48];

	ARG_UNUSED(data);
	ARG_UNUSED(len);

	runtime_utc_day_result = bg9x_parse_time_response_argv(argc, argv,
						       &runtime_utc_day_value,
						       cclk_buf, sizeof(cclk_buf));
	return 0;
}

MODEM_CMD_DEFINE(on_cmd_atcmdinfo_qlts)
{
	char qlts_buf[64];

	ARG_UNUSED(data);
	ARG_UNUSED(len);

	runtime_utc_day_result = bg9x_parse_time_response_argv(argc, argv,
						       &runtime_utc_day_value,
						       qlts_buf, sizeof(qlts_buf));
	return 0;
}

int bg9x_utc_day_get(uint32_t *utc_day)
{
	struct modem_cmd cclk_cmd[] = {
		MODEM_CMD_ARGS_MAX("+CCLK: ", on_cmd_atcmdinfo_cclk, 1U, 2U, ","),
	};
	struct modem_cmd qlts_cmd[] = {
		MODEM_CMD_ARGS_MAX("+QLTS: ", on_cmd_atcmdinfo_qlts, 1U, 3U, ","),
	};
	int ret;

	if (utc_day == NULL) {
		return -EINVAL;
	}

	ret = modem_ensure_ready();
	if (ret < 0) {
		return ret;
	}

	runtime_utc_day_result = -ENODATA;
	runtime_utc_day_value = 0U;

	if (runtime_qlts_state != BG9X_QLTS_STATE_UNAVAILABLE) {
		ret = modem_cmd_send(&mctx.iface, &mctx.cmd_handler,
				     qlts_cmd, ARRAY_SIZE(qlts_cmd), "AT+QLTS=1",
				     &mdata.sem_response, MDM_CMD_TIMEOUT);
		if (ret == 0 && runtime_utc_day_result == 0) {
			runtime_qlts_state = BG9X_QLTS_STATE_SUPPORTED;
			*utc_day = runtime_utc_day_value;
			return 0;
		}

		runtime_qlts_state = BG9X_QLTS_STATE_UNAVAILABLE;
		LOG_INF("AT+QLTS unavailable ret=%d parse=%d; using AT+CCLK?",
			ret, runtime_utc_day_result);
	}

	runtime_utc_day_result = -ENODATA;
	runtime_utc_day_value = 0U;

	ret = modem_cmd_send(&mctx.iface, &mctx.cmd_handler,
			     cclk_cmd, ARRAY_SIZE(cclk_cmd), "AT+CCLK?", &mdata.sem_response,
			     MDM_CMD_TIMEOUT);
	if (ret < 0) {
		return ret;
	}

	if (runtime_utc_day_result < 0) {
		return runtime_utc_day_result;
	}

	*utc_day = runtime_utc_day_value;
	return 0;
}

static K_KERNEL_STACK_DEFINE(modem_rx_stack, CONFIG_MODEM_QUECTEL_BG9X_RX_STACK_SIZE);
static K_KERNEL_STACK_DEFINE(modem_workq_stack, CONFIG_MODEM_QUECTEL_BG9X_RX_WORKQ_STACK_SIZE);
NET_BUF_POOL_DEFINE(mdm_recv_pool, MDM_RECV_MAX_BUF, MDM_RECV_BUF_SIZE, 0, NULL);

static const struct gpio_dt_spec power_gpio = GPIO_DT_SPEC_INST_GET(0, mdm_power_gpios);
#if DT_INST_NODE_HAS_PROP(0, mdm_reset_gpios)
static const struct gpio_dt_spec reset_gpio = GPIO_DT_SPEC_INST_GET(0, mdm_reset_gpios);
#endif
#if DT_INST_NODE_HAS_PROP(0, mdm_dtr_gpios)
static const struct gpio_dt_spec dtr_gpio = GPIO_DT_SPEC_INST_GET(0, mdm_dtr_gpios);
#endif
#if DT_INST_NODE_HAS_PROP(0, mdm_status_gpios)
static const struct gpio_dt_spec status_gpio = GPIO_DT_SPEC_INST_GET(0, mdm_status_gpios);
#endif
#if DT_INST_NODE_HAS_PROP(0, mdm_ap_ready_gpios)
static const struct gpio_dt_spec ap_ready_gpio = GPIO_DT_SPEC_INST_GET(0, mdm_ap_ready_gpios);
#endif
#if DT_INST_NODE_HAS_PROP(0, mdm_psm_gpios)
static const struct gpio_dt_spec psm_gpio = GPIO_DT_SPEC_INST_GET(0, mdm_psm_gpios);
#endif
#if DT_INST_NODE_HAS_PROP(0, mdm_ri_gpios)
static const struct gpio_dt_spec ri_gpio = GPIO_DT_SPEC_INST_GET(0, mdm_ri_gpios);
#endif
#if DT_INST_NODE_HAS_PROP(0, mdm_wdisable_gpios)
static const struct gpio_dt_spec wdisable_gpio = GPIO_DT_SPEC_INST_GET(0, mdm_wdisable_gpios);
#endif

#if defined(CONFIG_DNS_RESOLVER)
static struct zsock_addrinfo dns_result;
static struct sockaddr_in dns_result_addr;
static char dns_result_canonname[128];
#endif

static inline int digits(int n)
{
	int count = 0;

	while (n != 0) {
		n /= 10;
		++count;
	}

	return count;
}

static inline uint32_t hash32(char *str, int len)
{
#define HASH_MULTIPLIER		37

	uint32_t h = 0;
	int i;

	for (i = 0; i < len; ++i) {
		h = (h * HASH_MULTIPLIER) + str[i];
	}

	return h;
}

static inline uint8_t *modem_get_mac(const struct device *dev)
{
	struct modem_data *data = dev->data;
	uint32_t hash_value;

	data->mac_addr[0] = 0x00;
	data->mac_addr[1] = 0x10;

	/* use IMEI for mac_addr */
	hash_value = hash32(mdata.mdm_imei, strlen(mdata.mdm_imei));

	UNALIGNED_PUT(hash_value, (uint32_t *)(data->mac_addr + 2));

	return data->mac_addr;
}

/* Func: modem_atoi
 * Desc: Convert string to long integer, but handle errors
 */
static int modem_atoi(const char *s, const int err_value,
		      const char *desc, const char *func)
{
	int   ret;
	char  *endptr;

	ret = (int)strtol(s, &endptr, 10);
	if (!endptr || *endptr != '\0') {
		LOG_ERR("bad %s '%s' in %s", s, desc,
			func);
		return err_value;
	}

	return ret;
}

static inline int find_len(char *data)
{
	char buf[10] = {0};
	int  i;

	for (i = 0; i < 10; i++) {
		if (data[i] == '\r') {
			break;
		}

		buf[i] = data[i];
	}

	return ATOI(buf, 0, "rx_buf");
}

/* Func: on_cmd_sockread_common
 * Desc: Function to successfully read data from the modem on a given socket.
 */
static int on_cmd_sockread_common(int socket_fd,
				  struct modem_cmd_handler_data *data,
				  uint16_t len)
{
	struct modem_socket	 *sock = NULL;
	struct socket_read_data	 *sock_data;
	int ret, i;
	int socket_data_length;
	int bytes_to_skip;

	if (!len) {
		LOG_ERR("Invalid length, Aborting!");
		return -EAGAIN;
	}

	/* Make sure we still have buf data */
	if (!data->rx_buf) {
		LOG_ERR("Incorrect format! Ignoring data!");
		return -EINVAL;
	}

	socket_data_length = find_len(data->rx_buf->data);

	/* No (or not enough) data available on the socket. */
	bytes_to_skip = digits(socket_data_length) + 2 + 4;
	if (socket_data_length <= 0) {
		LOG_ERR("Length problem (%d).  Aborting!", socket_data_length);
		return -EAGAIN;
	}

	/* check to make sure we have all of the data. */
	if (net_buf_frags_len(data->rx_buf) < (socket_data_length + bytes_to_skip)) {
		LOG_DBG("Not enough data -- wait!");
		return -EAGAIN;
	}

	/* Skip "len" and CRLF */
	bytes_to_skip = digits(socket_data_length) + 2;
	for (i = 0; i < bytes_to_skip; i++) {
		net_buf_pull_u8(data->rx_buf);
	}

	if (!data->rx_buf->len) {
		data->rx_buf = net_buf_frag_del(NULL, data->rx_buf);
	}

	sock = modem_socket_from_fd(&mdata.socket_config, socket_fd);
	if (!sock) {
		LOG_ERR("Socket not found! (%d)", socket_fd);
		ret = -EINVAL;
		goto exit;
	}

	sock_data = (struct socket_read_data *)sock->data;
	if (!sock_data) {
		LOG_ERR("Socket data not found! Skip handling (%d)", socket_fd);
		ret = -EINVAL;
		goto exit;
	}

	ret = net_buf_linearize(sock_data->recv_buf, sock_data->recv_buf_len,
				data->rx_buf, 0, (uint16_t)socket_data_length);
	data->rx_buf = net_buf_skip(data->rx_buf, ret);
	sock_data->recv_read_len = ret;
	if (ret != socket_data_length) {
		LOG_ERR("Total copied data is different then received data!"
			" copied:%d vs. received:%d", ret, socket_data_length);
		ret = -EINVAL;
	}

exit:
	/* remove packet from list (ignore errors) */
	(void)modem_socket_packet_size_update(&mdata.socket_config, sock,
					      -socket_data_length);

	/* don't give back semaphore -- OK to follow */
	return ret;
}

/* Func: socket_close
 * Desc: Function to close the given socket descriptor.
 */
static int socket_close_cmd(struct modem_socket *sock)
{
	char buf[sizeof("AT+QICLOSE=##")] = {0};
	int  ret;

	snprintk(buf, sizeof(buf), "AT+QICLOSE=%d", sock->id);

#if DT_INST_NODE_HAS_PROP(0, mdm_status_gpios)
	if (!modem_runtime_line_asserted(&status_gpio)) {
		LOG_INF("STATUS low during socket close; skipping %s", buf);
		sock->is_connected = false;
		return 0;
	}
#endif

	/* Tell the modem to close the socket. */
	ret = modem_cmd_send(&mctx.iface, &mctx.cmd_handler,
			     NULL, 0U, buf,
			     &mdata.sem_response, K_SECONDS(2));
	if (ret < 0) {
		LOG_WRN("%s ret:%d; closing socket locally", buf, ret);
	}

	sock->is_connected = false;
	return ret;
}

static void socket_close(struct modem_socket *sock)
{
	(void)offload_socket_set_flags(sock, 0);
	(void)socket_close_cmd(sock);

	modem_socket_put(&mdata.socket_config, sock->sock_fd);
}

/* Handler: OK */
MODEM_CMD_DEFINE(on_cmd_ok)
{
	modem_cmd_handler_set_error(data, 0);
	k_sem_give(&mdata.sem_response);
	return 0;
}

/* Handler: ERROR */
MODEM_CMD_DEFINE(on_cmd_error)
{
	modem_cmd_handler_set_error(data, -EIO);
	k_sem_give(&mdata.sem_response);
	return 0;
}

/* Handler: +CME Error: <err>[0] */
MODEM_CMD_DEFINE(on_cmd_exterror)
{
	modem_cmd_handler_set_error(data, -EIO);
	k_sem_give(&mdata.sem_response);
	return 0;
}

/* Handler: +CSQ: <signal_power>[0], <qual>[1] */
MODEM_CMD_DEFINE(on_cmd_atcmdinfo_rssi_csq)
{
	int rssi = ATOI(argv[0], 0, "signal_power");

	/* Check the RSSI value. */
	if (rssi == 31) {
		mdata.mdm_rssi = -51;
	} else if (rssi >= 0 && rssi <= 31) {
		mdata.mdm_rssi = -114 + ((rssi * 2) + 1);
	} else {
		mdata.mdm_rssi = -1000;
	}

	LOG_INF("RSSI: %d", mdata.mdm_rssi);
	return 0;
}

/* Handler: +QIOPEN: <connect_id>[0], <err>[1] */
MODEM_CMD_DEFINE(on_cmd_atcmdinfo_sockopen)
{
	int err = ATOI(argv[1], 0, "sock_err");

	LOG_INF("AT+QIOPEN: %d", err);
	modem_cmd_handler_set_error(data, err);
	k_sem_give(&mdata.sem_sock_conn);

	return 0;
}

/* Handler: <manufacturer> */
MODEM_CMD_DEFINE(on_cmd_atcmdinfo_manufacturer)
{
	size_t out_len = net_buf_linearize(mdata.mdm_manufacturer,
					   sizeof(mdata.mdm_manufacturer) - 1,
					   data->rx_buf, 0, len);
	mdata.mdm_manufacturer[out_len] = '\0';
	LOG_INF("Manufacturer: %s", mdata.mdm_manufacturer);
	return 0;
}

/* Handler: <model> */
MODEM_CMD_DEFINE(on_cmd_atcmdinfo_model)
{
	size_t out_len = net_buf_linearize(mdata.mdm_model,
					   sizeof(mdata.mdm_model) - 1,
					   data->rx_buf, 0, len);
	mdata.mdm_model[out_len] = '\0';

	/* Log the received information. */
	LOG_INF("Model: %s", mdata.mdm_model);
	return 0;
}

/* Handler: <rev> */
MODEM_CMD_DEFINE(on_cmd_atcmdinfo_revision)
{
	size_t out_len = net_buf_linearize(mdata.mdm_revision,
					   sizeof(mdata.mdm_revision) - 1,
					   data->rx_buf, 0, len);
	mdata.mdm_revision[out_len] = '\0';

	/* Log the received information. */
	LOG_INF("Revision: %s", mdata.mdm_revision);
	return 0;
}

/* Handler: <IMEI> */
MODEM_CMD_DEFINE(on_cmd_atcmdinfo_imei)
{
	size_t out_len = net_buf_linearize(mdata.mdm_imei,
					   sizeof(mdata.mdm_imei) - 1,
					   data->rx_buf, 0, len);
	mdata.mdm_imei[out_len] = '\0';

	/* Log the received information. */
	LOG_INF("IMEI: %s", mdata.mdm_imei);
	return 0;
}

#if defined(CONFIG_MODEM_SIM_NUMBERS)
/* Handler: <IMSI> */
MODEM_CMD_DEFINE(on_cmd_atcmdinfo_imsi)
{
	size_t	out_len = net_buf_linearize(mdata.mdm_imsi,
					    sizeof(mdata.mdm_imsi) - 1,
					    data->rx_buf, 0, len);
	mdata.mdm_imsi[out_len] = '\0';

	/* Log the received information. */
	LOG_INF("IMSI: %s", mdata.mdm_imsi);
	return 0;
}

/* Handler: <ICCID> */
MODEM_CMD_DEFINE(on_cmd_atcmdinfo_iccid)
{
	size_t out_len;
	char   *p;

	out_len = net_buf_linearize(mdata.mdm_iccid, sizeof(mdata.mdm_iccid) - 1,
				    data->rx_buf, 0, len);
	mdata.mdm_iccid[out_len] = '\0';

	/* Skip over the +CCID bit, which modems omit. */
	if (mdata.mdm_iccid[0] == '+') {
		p = strchr(mdata.mdm_iccid, ' ');
		if (p) {
			out_len = strlen(p + 1);
			memmove(mdata.mdm_iccid, p + 1, len + 1);
		}
	}

	LOG_INF("ICCID: %s", mdata.mdm_iccid);
	return 0;
}
#endif /* #if defined(CONFIG_MODEM_SIM_NUMBERS) */

/* Handler: TX Ready */
MODEM_CMD_DIRECT_DEFINE(on_cmd_tx_ready)
{
	k_sem_give(&mdata.sem_tx_ready);
	return len;
}

/* Handler: SEND OK */
MODEM_CMD_DEFINE(on_cmd_send_ok)
{
	modem_cmd_handler_set_error(data, 0);
	k_sem_give(&mdata.sem_response);

	return 0;
}

/* Handler: SEND FAIL */
MODEM_CMD_DEFINE(on_cmd_send_fail)
{
	mdata.sock_written = 0;
	modem_cmd_handler_set_error(data, -EIO);
	k_sem_give(&mdata.sem_response);

	return 0;
}

/* Handler: Read data */
MODEM_CMD_DEFINE(on_cmd_sock_readdata)
{
	return on_cmd_sockread_common(mdata.sock_fd, data, len);
}

/* Handler: Data receive indication. */
MODEM_CMD_DEFINE(on_cmd_unsol_recv)
{
	struct modem_socket *sock;
	int		     sock_id;
	int		     ret;

	sock_id = ATOI(argv[0], 0, "sock_id");

	/* +QIURC reports the modem connect ID, not the Zephyr fd. */
	sock = modem_socket_from_id(&mdata.socket_config, sock_id);
	if (!sock) {
		return 0;
	}

	/* The modem only tells us that data exists, not how much. Queue a
	 * dummy packet size so modem_socket poll/read paths become readable.
	 */
	ret = modem_socket_packet_size_update(&mdata.socket_config, sock, 1);
	if (ret < 0) {
		LOG_ERR("Failed to queue recv readiness for socket id %d: %d", sock_id, ret);
		return 0;
	}

	/* Data ready indication. */
	LOG_DBG("Data Receive Indication for socket id: %d fd: %d", sock_id, sock->sock_fd);
	modem_socket_data_ready(&mdata.socket_config, sock);

	return 0;
}

/* Handler: Socket Close Indication. */
MODEM_CMD_DEFINE(on_cmd_unsol_close)
{
	struct modem_socket *sock;
	int		     sock_id;

	sock_id = ATOI(argv[0], 0, "sock_id");
	sock	= modem_socket_from_id(&mdata.socket_config, sock_id);
	if (!sock) {
		return 0;
	}

	LOG_INF("Socket Close Indication for socket id: %d fd: %d", sock_id, sock->sock_fd);

	/* Tell the modem to close the socket. */
	socket_close(sock);
	LOG_INF("Socket Closed id: %d", sock_id);
	return 0;
}

#if defined(CONFIG_DNS_RESOLVER)
static int modem_dns_complete(int result_code)
{
	mdata.dns_result = result_code;
	mdata.dns_query_active = false;
	k_sem_give(&mdata.sem_dns);
	return 0;
}

static int modem_dns_parse_ipv4(const char *src)
{
	char ip_buf[NET_IPV4_ADDR_LEN] = {0};
	size_t copy_len = MIN(strlen(src), sizeof(ip_buf) - 1);

	memcpy(ip_buf, src, copy_len);
	ip_buf[copy_len] = '\0';

	if (ip_buf[0] == '"') {
		memmove(ip_buf, ip_buf + 1, strlen(ip_buf));
	}

	char *quote = strchr(ip_buf, '"');
	if (quote != NULL) {
		*quote = '\0';
	}

	if (net_addr_pton(AF_INET, ip_buf, &dns_result_addr.sin_addr) != 0) {
		return -EINVAL;
	}

	strncpy(dns_result_canonname, ip_buf, sizeof(dns_result_canonname) - 1);
	dns_result_canonname[sizeof(dns_result_canonname) - 1] = '\0';
	return 0;
}

/* Handler: +QIURC: "dnsgip",... */
MODEM_CMD_DEFINE(on_cmd_unsol_dnsgip)
{
	int ret;
	const char *arg0 = (argc >= 1) ? (const char *)argv[0] : "";
	const char *arg1 = (argc >= 2) ? (const char *)argv[1] : "";
	const char *arg2 = (argc >= 3) ? (const char *)argv[2] : "";

	LOG_DBG("DNS URC argc=%u a0='%s' a1='%s' a2='%s'",
		argc,
		arg0, arg1, arg2);

	if (!mdata.dns_query_active) {
		return 0;
	}

	if (argc >= 1 && arg0[0] == '"') {
		ret = modem_dns_parse_ipv4(arg0);
		if (ret < 0) {
			LOG_WRN("Malformed DNS argv IP response '%s'", arg0);
			return modem_dns_complete(DNS_EAI_FAIL);
		}

		return modem_dns_complete(0);
	}

	if (argc >= 1 && isdigit(arg0[0])) {
		ret = ATOI(arg0, -1, "dns_result");
		mdata.dns_modem_result = ret;
		if (ret == 564 || ret == 568) {
			LOG_WRN("DNS URC busy result %d, waiting for follow-up", ret);
			return 0;
		}
		if (ret != 0) {
			LOG_WRN("DNS lookup failed with modem result %d", ret);
			return modem_dns_complete(DNS_EAI_NONAME);
		}

		if (argc >= 3) {
			ret = modem_dns_parse_ipv4((const char *)argv[2]);
			if (ret == 0) {
				return modem_dns_complete(0);
			}
		}

		/* Success line without inline IP; wait for the follow-up IP line. */
		return 0;
	}

	if (data->rx_buf == NULL) {
		return modem_dns_complete(DNS_EAI_FAIL);
	}

	char ip_buf[NET_IPV4_ADDR_LEN + 2] = {0};
	size_t out_len = net_buf_linearize(ip_buf, sizeof(ip_buf) - 1,
					   data->rx_buf, 0, len);
	ip_buf[out_len] = '\0';
	ret = modem_dns_parse_ipv4(ip_buf);
	if (ret < 0) {
		LOG_WRN("Malformed DNS IP response '%s'", ip_buf);
		return modem_dns_complete(DNS_EAI_FAIL);
	}

	return modem_dns_complete(0);
}
#endif

MODEM_CMD_DEFINE(on_cmd_atcmdinfo_qiact)
{
	int cid;
	int state;

	if (argc < 2) {
		return 0;
	}

	cid = ATOI(argv[0], -1, "pdp_cid");
	state = ATOI(argv[1], -1, "pdp_state");
	if (cid == 1 && state == 1) {
		mdata.pdp_active = true;
	}

	return 0;
}

/* Handler: Modem initialization ready. */
MODEM_CMD_DEFINE(on_cmd_unsol_rdy)
{
	if (mdata.setup_complete) {
		mdata.runtime_wake_pending = true;
	}
	k_sem_give(&mdata.sem_response);
	return 0;
}

static int modem_runtime_prepare(void);
static int modem_runtime_emergency_reinit(void);
static int modem_setup(void);
static void modem_rssi_query_work(struct k_work *work);
static int modem_pdp_context_activate(void);
static int modem_runtime_restore(void);
static int bg9x_gps_send_cmd(const char *cmd, k_timeout_t timeout);
static int bg9x_gps_prepare_command_path(void);
static bool cycle_begin_prepared;

static int modem_ensure_ready(void)
{
	if (mdata.gps_active) {
		LOG_DBG("Modem ensure_ready skipped: GPS session active");
		return -EBUSY;
	}

	if (mdata.setup_complete) {
		if (mdata.runtime_wake_pending) {
			LOG_INF("Runtime wake pending before modem command");
			return modem_runtime_prepare();
		}

		return 0;
	}

	LOG_WRN("Modem setup incomplete, attempting setup on demand");
	return modem_setup();
}

static int bg9x_gps_send_cmd(const char *cmd, k_timeout_t timeout)
{
	return modem_cmd_send(&mctx.iface, &mctx.cmd_handler,
			      NULL, 0U, cmd, &mdata.sem_response, timeout);
}

static int bg9x_gps_prepare_command_path(void)
{
	int ret;

	if (cycle_begin_prepared) {
		/* bg9x_cycle_begin() already proved the modem is awake and AT-ready
		 * for this active cycle. Avoid re-running the wake escalation path
		 * from GPS begin and only give the command path a short settle.
		 */
		k_sleep(K_MSEC(250));
		return 0;
	}

	ret = modem_runtime_at_ping(K_SECONDS(2));
	if (ret == 0) {
		/* A single AT echo can succeed a little before the modem is
		 * reliably ready for the GPS scaffold command sequence.
		 */
		k_sleep(K_MSEC(250));
		return 0;
	}

	LOG_WRN("GPS scaffold AT-ready confirmation failed: %d; forcing runtime prepare", ret);
	mdata.runtime_wake_pending = true;
	ret = modem_runtime_prepare();
	if (ret < 0) {
		return ret;
	}

	ret = modem_runtime_at_ping(K_SECONDS(2));
	if (ret < 0) {
		LOG_WRN("GPS scaffold AT-ready confirmation still failed after runtime prepare: %d",
			ret);
		return ret;
	}

	k_sleep(K_MSEC(250));
	return 0;
}

/* Parse ddmm.mmmm[N/S] or dddmm.mmmm[E/W] to decimal degrees */
static double gps_parse_coord(const char *str, bool is_longitude)
{
	char buf[20];
	size_t len;
	int deg_digits = is_longitude ? 3 : 2;
	double degrees;
	double minutes;
	char *endptr;
	char direction;

	if (str == NULL) {
		return 0.0;
	}

	len = strlen(str);
	if (len < 4) {
		return 0.0;
	}

	/* Last char is N/S/E/W direction */
	direction = str[len - 1];

	/* Copy without direction char */
	if (len - 1 >= sizeof(buf)) {
		return 0.0;
	}
	memcpy(buf, str, len - 1);
	buf[len - 1] = '\0';

	/* Parse degrees (first 2 or 3 chars) */
	char deg_buf[4] = {0};
	memcpy(deg_buf, buf, deg_digits);
	degrees = strtod(deg_buf, NULL);

	/* Parse minutes (rest) */
	minutes = strtod(buf + deg_digits, &endptr);
	degrees += minutes / 60.0;

	if (direction == 'S' || direction == 'W') {
		degrees = -degrees;
	}

	return degrees;
}

/* Parse QGPSLOC response:
 * +QGPSLOC: <UTC>,<lat>,<lon>,<HDOP>,<alt>,<fix>,<CoG>,<spkm>,<spkn>,<date>,<nsat>
 * The modem_cmd framework splits on comma, giving us argv[0]=UTC, argv[1]=lat, etc.
 */
MODEM_CMD_DEFINE(on_cmd_gps_qgpsloc)
{
	ARG_UNUSED(data);
	ARG_UNUSED(len);

	if (argc < 11) {
		LOG_WRN("QGPSLOC too few fields: %d", argc);
		return 0;
	}

	/* UTC time: hhmmss.sss */
	if (strlen(argv[0]) >= 6) {
		gps_pending_fix.utc_hour = (argv[0][0] - '0') * 10 + (argv[0][1] - '0');
		gps_pending_fix.utc_min = (argv[0][2] - '0') * 10 + (argv[0][3] - '0');
		gps_pending_fix.utc_sec = (argv[0][4] - '0') * 10 + (argv[0][5] - '0');
		if (strlen(argv[0]) > 7 && argv[0][6] == '.') {
			gps_pending_fix.utc_ms = (uint16_t)strtoul(argv[0] + 7, NULL, 10);
		}
	}

	/* Latitude: ddmm.mmmmN/S */
	gps_pending_fix.latitude = gps_parse_coord(argv[1], false);

	/* Longitude: dddmm.mmmmE/W */
	gps_pending_fix.longitude = gps_parse_coord(argv[2], true);

	/* HDOP */
	gps_pending_fix.hdop = (float)strtod(argv[3], NULL);

	/* Altitude */
	gps_pending_fix.altitude = (float)strtod(argv[4], NULL);

	/* Fix type */
	gps_pending_fix.fix_type = (uint8_t)strtoul(argv[5], NULL, 10);

	/* Course over ground */
	gps_pending_fix.course = (float)strtod(argv[6], NULL);

	/* Speed km/h */
	gps_pending_fix.speed_kmh = (float)strtod(argv[7], NULL);

	/* Speed knots */
	gps_pending_fix.speed_knots = (float)strtod(argv[8], NULL);

	/* Date: ddmmyy */
	if (strlen(argv[9]) >= 6) {
		gps_pending_fix.day = (argv[9][0] - '0') * 10 + (argv[9][1] - '0');
		gps_pending_fix.month = (argv[9][2] - '0') * 10 + (argv[9][3] - '0');
		int yy = (argv[9][4] - '0') * 10 + (argv[9][5] - '0');
		gps_pending_fix.year = (yy >= 80) ? (1900 + yy) : (2000 + yy);
	}

	/* Sats in use */
	gps_pending_fix.sats_in_use = (uint8_t)strtoul(argv[10], NULL, 10);

	gps_pending_fix.valid = (gps_pending_fix.fix_type >= 2);
	gps_fix_received = true;

	return 0;
}

/* Parse NMEA GSA sentence for PDOP/VDOP:
 * +QGPSGNMEA: $GPGSA,A,3,04,05,...,PDOP,HDOP,VDOP*xx
 * Response comes as one argv with the full NMEA sentence.
 */
MODEM_CMD_DEFINE(on_cmd_gps_nmea)
{
	ARG_UNUSED(data);
	ARG_UNUSED(len);

	if (argc < 1 || argv[0] == NULL) {
		return 0;
	}

	/* Reconstruct full NMEA from argv (framework may have split on commas) */
	size_t offset = 0;
	for (int i = 0; i < argc && offset < sizeof(gps_nmea_buf) - 2; i++) {
		if (i > 0 && offset < sizeof(gps_nmea_buf) - 1) {
			gps_nmea_buf[offset++] = ',';
		}
		size_t flen = strlen(argv[i]);
		if (offset + flen >= sizeof(gps_nmea_buf)) {
			break;
		}
		memcpy(gps_nmea_buf + offset, argv[i], flen);
		offset += flen;
	}
	gps_nmea_buf[offset] = '\0';

	return 0;
}

static void gps_parse_gsa(const char *nmea)
{
	/* $GPGSA,A,3,prn1,...,prn12,PDOP,HDOP,VDOP*xx */
	/* Fields: 0=type, 1=mode, 2=fix, 3-14=PRNs, 15=PDOP, 16=HDOP, 17=VDOP */
	char buf[256];
	char *fields[20];
	int nfields = 0;
	char *p;
	char *star;

	strncpy(buf, nmea, sizeof(buf) - 1);
	buf[sizeof(buf) - 1] = '\0';

	/* Strip checksum */
	star = strchr(buf, '*');
	if (star) {
		*star = '\0';
	}

	p = buf;
	while (p && nfields < 20) {
		fields[nfields++] = p;
		p = strchr(p, ',');
		if (p) {
			*p++ = '\0';
		}
	}

	if (nfields >= 17) {
		gps_pending_fix.pdop = (float)strtod(fields[15], NULL);
		/* fields[16] is HDOP — we already have it from QGPSLOC */
		if (nfields >= 18) {
			gps_pending_fix.vdop = (float)strtod(fields[17], NULL);
		}
	}
}

static void gps_parse_gsv(const char *nmea)
{
	/* $GPGSV,total_msgs,msg_num,total_sats,[PRN,elev,azimuth,SNR]x4*xx */
	char buf[256];
	char *fields[24];
	int nfields = 0;
	char *p;
	char *star;

	strncpy(buf, nmea, sizeof(buf) - 1);
	buf[sizeof(buf) - 1] = '\0';

	star = strchr(buf, '*');
	if (star) {
		*star = '\0';
	}

	p = buf;
	while (p && nfields < 24) {
		fields[nfields++] = p;
		p = strchr(p, ',');
		if (p) {
			*p++ = '\0';
		}
	}

	if (nfields >= 4) {
		gps_pending_fix.sats_in_view = (uint8_t)strtoul(fields[3], NULL, 10);
	}

	/* Parse sat details from this sentence (up to 4 per GSV sentence) */
	int msg_num = (nfields >= 3) ? (int)strtoul(fields[2], NULL, 10) : 0;
	int base_idx = (msg_num - 1) * 4;

	for (int i = 0; i < 4 && (4 + i * 4 + 3) < nfields; i++) {
		int idx = base_idx + i;
		if (idx >= 16) {
			break;
		}
		int fi = 4 + i * 4;
		gps_pending_fix.sat_detail[idx].prn =
			(uint8_t)strtoul(fields[fi], NULL, 10);
		gps_pending_fix.sat_detail[idx].elevation =
			(uint8_t)strtoul(fields[fi + 1], NULL, 10);
		gps_pending_fix.sat_detail[idx].azimuth =
			(uint16_t)strtoul(fields[fi + 2], NULL, 10);
		gps_pending_fix.sat_detail[idx].snr =
			(uint8_t)strtoul(fields[fi + 3], NULL, 10);
		if (idx >= gps_pending_fix.sat_count) {
			gps_pending_fix.sat_count = idx + 1;
		}
	}
}

static void gps_parse_vtg(const char *nmea)
{
	/* $GPVTG,true_course,T,mag_course,M,speed_kn,N,speed_km,K*xx */
	char buf[128];
	char *fields[12];
	int nfields = 0;
	char *p;
	char *star;

	strncpy(buf, nmea, sizeof(buf) - 1);
	buf[sizeof(buf) - 1] = '\0';

	star = strchr(buf, '*');
	if (star) {
		*star = '\0';
	}

	p = buf;
	while (p && nfields < 12) {
		fields[nfields++] = p;
		p = strchr(p, ',');
		if (p) {
			*p++ = '\0';
		}
	}

	if (nfields >= 4 && strlen(fields[3]) > 0) {
		gps_pending_fix.magnetic_course = (float)strtod(fields[3], NULL);
	}
}

static void gps_parse_rmc(const char *nmea)
{
	/* $GPRMC,...,mag_var,E/W*xx — fields 10 and 11 */
	char buf[200];
	char *fields[16];
	int nfields = 0;
	char *p;
	char *star;

	strncpy(buf, nmea, sizeof(buf) - 1);
	buf[sizeof(buf) - 1] = '\0';

	star = strchr(buf, '*');
	if (star) {
		*star = '\0';
	}

	p = buf;
	while (p && nfields < 16) {
		fields[nfields++] = p;
		p = strchr(p, ',');
		if (p) {
			*p++ = '\0';
		}
	}

	if (nfields >= 12 && strlen(fields[10]) > 0) {
		gps_pending_fix.magnetic_variation = (float)strtod(fields[10], NULL);
		if (nfields >= 12 && fields[11][0] == 'W') {
			gps_pending_fix.magnetic_variation = -gps_pending_fix.magnetic_variation;
		}
	}

	/* Store raw RMC */
	strncpy(gps_pending_fix.nmea_rmc, nmea,
		sizeof(gps_pending_fix.nmea_rmc) - 1);
	gps_pending_fix.nmea_rmc[sizeof(gps_pending_fix.nmea_rmc) - 1] = '\0';
}

static int bg9x_gps_query_nmea(const char *sentence_type)
{
	struct modem_cmd nmea_cmd[] = {
		MODEM_CMD_ARGS_MAX("+QGPSGNMEA: ", on_cmd_gps_nmea, 1U, 1U, ""),
	};
	char cmd_buf[32];
	int ret;

	snprintk(cmd_buf, sizeof(cmd_buf), "AT+QGPSGNMEA=\"%s\"", sentence_type);
	gps_nmea_buf[0] = '\0';

	ret = modem_cmd_send(&mctx.iface, &mctx.cmd_handler,
			     nmea_cmd, ARRAY_SIZE(nmea_cmd), cmd_buf,
			     &mdata.sem_response, MDM_CMD_TIMEOUT);
	return ret;
}

/* Parse +QENG: "servingcell",...  response for cell info */
MODEM_CMD_DEFINE(on_cmd_cell_info)
{
	ARG_UNUSED(data);
	ARG_UNUSED(len);

	/*
	 * +QENG: "servingcell","CONNECT","CAT-M","FDD",
	 *   MCC,MNC,cellID(hex),PCI,EARFCN,band,UL_BW,DL_BW,TAC,RSRP,RSRQ,RSSI,SINR,...
	 * After prefix strip and comma split:
	 *   argv[0]="servingcell", argv[1]="CONNECT", argv[2]="CAT-M", argv[3]="FDD",
	 *   argv[4]=MCC, argv[5]=MNC, argv[6]=cellID, argv[7]=PCI, argv[8]=EARFCN,
	 *   argv[9]=band, argv[10]=UL_BW, argv[11]=DL_BW, argv[12]=TAC,
	 *   argv[13]=RSRP, argv[14]=RSRQ, argv[15]=RSSI, argv[16]=SINR
	 */
	/*
	 * BG95 +QENG: "servingcell" response (eMTC/Cat-M1):
	 * argv[0]="servingcell" [1]="NOCONN"/"CONNECT" [2]="eMTC" [3]="FDD"
	 * [4]=MCC [5]=MNC [6]=cellID(hex) [7]=PCI [8]=EARFCN [9]=band
	 * [10]=UL_BW [11]=DL_BW [12]=TAC(hex) [13]=RSRP [14]=RSRQ
	 * [15]=RSSI [16]=SINR
	 * Requires CONFIG_MODEM_CMD_HANDLER_MAX_PARAM_COUNT >= 17
	 */
	if (argc < 6) {
		LOG_WRN("QENG too few fields: %d", argc);
		return 0;
	}

	/* RAT (argv[2]) */
	strncpy(cell_pending_info.rat, argv[2], sizeof(cell_pending_info.rat) - 1);
	cell_pending_info.rat[sizeof(cell_pending_info.rat) - 1] = '\0';

	cell_pending_info.mcc = (uint16_t)strtoul(argv[4], NULL, 10);
	cell_pending_info.mnc = (uint16_t)strtoul(argv[5], NULL, 10);

	if (argc >= 10) {
		cell_pending_info.cell_id = (uint32_t)strtoul(argv[6], NULL, 16);
		cell_pending_info.pci = (uint16_t)strtoul(argv[7], NULL, 10);
		cell_pending_info.earfcn = (uint32_t)strtoul(argv[8], NULL, 10);
		cell_pending_info.band = (uint8_t)strtoul(argv[9], NULL, 10);
	}
	if (argc >= 13) {
		cell_pending_info.tac = (uint16_t)strtoul(argv[12], NULL, 16);
	}
	if (argc >= 17) {
		cell_pending_info.rsrp = (int16_t)strtol(argv[13], NULL, 10);
		cell_pending_info.rsrq = (int16_t)strtol(argv[14], NULL, 10);
		cell_pending_info.rssi = (int16_t)strtol(argv[15], NULL, 10);
		cell_pending_info.sinr = (int16_t)strtol(argv[16], NULL, 10);
	}
	cell_pending_info.valid = true;
	cell_info_received = true;

	return 0;
}

int bg9x_cell_info_get(struct bg9x_cell_info *info)
{
	struct modem_cmd cmd[] = {
		MODEM_CMD_ARGS_MAX("+QENG: ", on_cmd_cell_info, 4U, 20U, ","),
	};
	int ret;

	if (info == NULL) {
		return -EINVAL;
	}

	memset(info, 0, sizeof(*info));

	ret = modem_ensure_ready();
	if (ret < 0) {
		return ret;
	}

	cell_info_received = false;
	memset(&cell_pending_info, 0, sizeof(cell_pending_info));

	ret = modem_cmd_send(&mctx.iface, &mctx.cmd_handler,
			     cmd, ARRAY_SIZE(cmd), "AT+QENG=\"servingcell\"",
			     &mdata.sem_response, MDM_CMD_TIMEOUT);
	if (ret < 0) {
		return ret;
	}

	if (!cell_info_received) {
		return -ENODATA;
	}

	memcpy(info, &cell_pending_info, sizeof(*info));
	return 0;
}

/* Network time from AT+CCLK? (QLTS unavailable on this network) */
static struct bg9x_network_time ntime_pending;
static bool ntime_received;

MODEM_CMD_DEFINE(on_cmd_ntime_cclk)
{
	char buf[48];
	size_t out = 0U;
	int year;
	int month;
	int day;
	int hour;
	int minute;
	int second;
	int offset_quarters;
	char sign;
	int matched;

	ARG_UNUSED(data);
	ARG_UNUSED(len);

	/* Reconstruct from argv */
	for (int i = 0; i < argc && out < sizeof(buf) - 2; i++) {
		if (i > 0 && out < sizeof(buf) - 1) {
			buf[out++] = ',';
		}
		size_t flen = strlen(argv[i]);
		if (out + flen >= sizeof(buf)) {
			break;
		}
		memcpy(buf + out, argv[i], flen);
		out += flen;
	}
	buf[out] = '\0';

	/* Strip quotes */
	{
		size_t j = 0;
		for (size_t i = 0; buf[i] != '\0' && j < sizeof(buf) - 1; i++) {
			if (buf[i] != '"') {
				buf[j++] = buf[i];
			}
		}
		buf[j] = '\0';
	}

	/* Parse: YY/MM/DD,HH:MM:SS+/-QQ */
	matched = sscanf(buf, "%d/%d/%d,%d:%d:%d%c%d",
			 &year, &month, &day, &hour, &minute, &second,
			 &sign, &offset_quarters);
	if (matched < 8) {
		return 0;
	}

	ntime_pending.year = (year >= 80) ? (1900 + year) : (2000 + year);
	ntime_pending.month = (uint8_t)month;
	ntime_pending.day = (uint8_t)day;
	ntime_pending.hour = (uint8_t)hour;
	ntime_pending.min = (uint8_t)minute;
	ntime_pending.sec = (uint8_t)second;
	ntime_pending.tz_quarter_hours = (sign == '-')
		? (int8_t)(-offset_quarters)
		: (int8_t)(offset_quarters);
	ntime_pending.valid = true;
	ntime_received = true;

	return 0;
}

int bg9x_network_time_get(struct bg9x_network_time *time)
{
	struct modem_cmd cmd[] = {
		MODEM_CMD_ARGS_MAX("+CCLK: ", on_cmd_ntime_cclk, 1U, 2U, ","),
	};
	int ret;

	if (time == NULL) {
		return -EINVAL;
	}

	memset(time, 0, sizeof(*time));

	ret = modem_ensure_ready();
	if (ret < 0) {
		return ret;
	}

	ntime_received = false;
	memset(&ntime_pending, 0, sizeof(ntime_pending));

	ret = modem_cmd_send(&mctx.iface, &mctx.cmd_handler,
			     cmd, ARRAY_SIZE(cmd), "AT+CCLK?",
			     &mdata.sem_response, MDM_CMD_TIMEOUT);
	if (ret < 0) {
		return ret;
	}

	if (!ntime_received) {
		return -ENODATA;
	}

	memcpy(time, &ntime_pending, sizeof(*time));
	return 0;
}

/* AT command passthrough for interactive testing via shell */
static char at_passthrough_buf[512];
static size_t at_passthrough_offset;
static bool bg9x_qistate_has_socket(const char *resp, int sock_id);
static int bg9x_recover_vanished_socket(struct modem_socket *sock);

MODEM_CMD_DEFINE(on_cmd_at_passthrough)
{
	ARG_UNUSED(data);
	ARG_UNUSED(len);

	/* Add newline between multiple response lines */
	if (at_passthrough_offset > 0 &&
	    at_passthrough_offset < sizeof(at_passthrough_buf) - 2) {
		at_passthrough_buf[at_passthrough_offset++] = '\n';
	}

	/* Prepend "+" since the match prefix was stripped */
	if (at_passthrough_offset < sizeof(at_passthrough_buf) - 1) {
		at_passthrough_buf[at_passthrough_offset++] = '+';
	}

	/* Reconstruct from argv with comma delimiters */
	for (int i = 0; i < argc; i++) {
		if (i > 0 && at_passthrough_offset <
		    sizeof(at_passthrough_buf) - 1) {
			at_passthrough_buf[at_passthrough_offset++] = ',';
		}
		size_t flen = strlen(argv[i]);

		if (at_passthrough_offset + flen >=
		    sizeof(at_passthrough_buf) - 1) {
			break;
		}
		memcpy(at_passthrough_buf + at_passthrough_offset,
		       argv[i], flen);
		at_passthrough_offset += flen;
	}
	at_passthrough_buf[at_passthrough_offset] = '\0';

	return 0;
}

static int bg9x_at_cmd_passthrough_internal(const char *cmd, char *resp, size_t resp_size,
					    bool prepare_on_timeout)
{
	struct modem_cmd match[] = {
		MODEM_CMD_ARGS_MAX("+", on_cmd_at_passthrough, 1U, 20U, ","),
	};
	int ret;

	if (cmd == NULL) {
		return -EINVAL;
	}

	ret = modem_ensure_ready();
	if (ret < 0) {
		return ret;
	}

	at_passthrough_buf[0] = '\0';
	at_passthrough_offset = 0;

	ret = modem_cmd_send(&mctx.iface, &mctx.cmd_handler,
			     match, ARRAY_SIZE(match), cmd,
			     &mdata.sem_response, MDM_CMD_TIMEOUT);
	if (ret == -ETIMEDOUT && prepare_on_timeout) {
		LOG_WRN("AT passthrough timeout for '%s'; retrying after runtime prepare", cmd);
		ret = modem_runtime_prepare();
		if (ret == 0) {
			at_passthrough_buf[0] = '\0';
			at_passthrough_offset = 0;
			ret = modem_cmd_send(&mctx.iface, &mctx.cmd_handler,
					     match, ARRAY_SIZE(match), cmd,
					     &mdata.sem_response, MDM_CMD_TIMEOUT);
		}
	}

	if (resp != NULL && resp_size > 0) {
		strncpy(resp, at_passthrough_buf, resp_size - 1);
		resp[resp_size - 1] = '\0';
	}

	return ret;
}

int bg9x_at_cmd_passthrough(const char *cmd, char *resp, size_t resp_size)
{
	return bg9x_at_cmd_passthrough_internal(cmd, resp, resp_size, true);
}

static bool psm_runtime_disabled;
static bool psm_resume_after_gps;
static bool gps_static_cfg_done;
static bool xtra_policy_enabled;
static uint8_t xtra_policy_retention_days = 3U;
static uint8_t xtra_policy_mode;
static bool xtra_policy_dirty = true;
static uint32_t psm_runtime_wake_seconds;
static bool psm_policy_dirty = true;

#define BG9X_XTRA_REFRESH_THRESHOLD_MINUTES (12U * 60U)

struct bg9x_psm_tau_unit {
	uint32_t seconds;
	uint8_t unit_bits;
};

static const struct bg9x_psm_tau_unit bg9x_psm_tau_units[] = {
	{ 2U, 0x03U },        /* 011 = 2 seconds */
	{ 30U, 0x04U },       /* 100 = 30 seconds */
	{ 60U, 0x05U },       /* 101 = 1 minute */
	{ 600U, 0x00U },      /* 000 = 10 minutes */
	{ 3600U, 0x01U },     /* 001 = 1 hour */
	{ 36000U, 0x02U },    /* 010 = 10 hours */
	{ 1152000U, 0x06U },  /* 110 = 320 hours */
};

static void bg9x_psm_encode_bits(uint8_t unit_bits, uint8_t value, char out[9])
{
	for (int bit = 0; bit < 3; bit++) {
		out[bit] = (unit_bits & (1U << (2 - bit))) ? '1' : '0';
	}

	for (int bit = 0; bit < 5; bit++) {
		out[3 + bit] = (value & (1U << (4 - bit))) ? '1' : '0';
	}

	out[8] = '\0';
}

static int bg9x_psm_encode_tau(uint32_t wake_seconds, char out[9], uint32_t *actual_seconds)
{
	uint32_t best_seconds = 0U;
	uint8_t best_unit = 0U;
	uint8_t best_value = 0U;
	uint32_t best_delta = UINT32_MAX;

	if (out == NULL) {
		return -EINVAL;
	}

	if (wake_seconds == 0U) {
		strncpy(out, CONFIG_MODEM_QUECTEL_BG9X_PSM_TAU, 9);
		if (actual_seconds != NULL) {
			*actual_seconds = 0U;
		}
		return 0;
	}

	for (size_t i = 0U; i < ARRAY_SIZE(bg9x_psm_tau_units); i++) {
		uint32_t unit_seconds = bg9x_psm_tau_units[i].seconds;
		uint32_t value = DIV_ROUND_UP(wake_seconds, unit_seconds);
		uint32_t encoded_seconds;
		uint32_t delta;

		if (value == 0U || value > 31U) {
			continue;
		}

		encoded_seconds = value * unit_seconds;
		delta = encoded_seconds - wake_seconds;
		if (delta < best_delta ||
		    (delta == best_delta && encoded_seconds < best_seconds)) {
			best_delta = delta;
			best_seconds = encoded_seconds;
			best_unit = bg9x_psm_tau_units[i].unit_bits;
			best_value = (uint8_t)value;
		}
	}

	if (best_seconds == 0U) {
		return -ERANGE;
	}

	bg9x_psm_encode_bits(best_unit, best_value, out);
	if (actual_seconds != NULL) {
		*actual_seconds = best_seconds;
	}

	return 0;
}

static int bg9x_apply_xtra_policy(void)
{
	int ret;
	bool autodownload;
	bool trigger_download = false;
	uint32_t valid_minutes = 0U;
	char resp[96];

	if (!xtra_policy_dirty) {
		return 0;
	}

	if (!xtra_policy_enabled) {
		/* Do not churn GNSS config every cycle when XTRA is disabled. */
		LOG_DBG("GPS XTRA policy applied enabled=0 retention=%u mode=%u autodl=0 (no-op)",
			xtra_policy_retention_days, xtra_policy_mode);
		xtra_policy_dirty = false;
		return 0;
	}

	switch (xtra_policy_retention_days) {
	case 1:
		ret = bg9x_gps_send_cmd("AT+QGPSCFG=\"xtrafilesize\",1", MDM_CMD_TIMEOUT);
		break;
	case 2:
		ret = bg9x_gps_send_cmd("AT+QGPSCFG=\"xtrafilesize\",2", MDM_CMD_TIMEOUT);
		break;
	case 3:
		ret = bg9x_gps_send_cmd("AT+QGPSCFG=\"xtrafilesize\",3", MDM_CMD_TIMEOUT);
		break;
	case 7:
		ret = bg9x_gps_send_cmd("AT+QGPSCFG=\"xtrafilesize\",7", MDM_CMD_TIMEOUT);
		break;
	default:
		ret = -EINVAL;
		break;
	}

	if (ret < 0) {
		LOG_WRN("GPS XTRA filesize=%u apply failed: %d",
			xtra_policy_retention_days, ret);
		return ret;
	}

	autodownload = (xtra_policy_mode == 1U);
	ret = bg9x_gps_send_cmd(autodownload ?
			"AT+QGPSCFG=\"xtra_autodownload\",1" :
			"AT+QGPSCFG=\"xtra_autodownload\",0",
			MDM_CMD_TIMEOUT);
	if (ret < 0) {
		LOG_WRN("GPS XTRA autodownload=%u apply failed: %d",
			autodownload ? 1U : 0U, ret);
		return ret;
	}

	if (xtra_policy_mode == 0U) {
		ret = bg9x_at_cmd_passthrough("AT+QGPSXTRADATA?", resp, sizeof(resp));
		if (ret < 0) {
			LOG_WRN("GPS XTRA validity query failed: %d", ret);
			return ret;
		}

		if (sscanf(resp, "+QGPSXTRADATA: %u", &valid_minutes) == 1) {
			if (valid_minutes <= BG9X_XTRA_REFRESH_THRESHOLD_MINUTES) {
				trigger_download = true;
			}
			LOG_INF("GPS XTRA validity=%umin threshold=%umin refresh=%u",
				valid_minutes, BG9X_XTRA_REFRESH_THRESHOLD_MINUTES,
				trigger_download ? 1U : 0U);
		} else {
			LOG_WRN("GPS XTRA validity parse failed: %s", resp);
			return -EBADMSG;
		}
	} else if (xtra_policy_mode == 2U) {
		trigger_download = true;
		LOG_INF("GPS XTRA force-refresh requested");
	}

	if (trigger_download) {
		ret = bg9x_gps_send_cmd("AT+QGPSCFG=\"xtra_download\",1",
					MDM_CMD_TIMEOUT);
		if (ret < 0) {
			LOG_WRN("GPS XTRA download trigger failed: %d", ret);
			return ret;
		}
	}

	LOG_INF("GPS XTRA policy applied enabled=1 retention=%u mode=%u autodl=%u refresh=%u",
		xtra_policy_retention_days,
		xtra_policy_mode,
		autodownload ? 1U : 0U,
		trigger_download ? 1U : 0U);

	xtra_policy_dirty = false;
	return 0;
}

static int bg9x_apply_psm_preference(const char *reason)
{
	int ret;
	char tau[9];
	char cmd[48];
	uint32_t applied_seconds = 0U;

	if (psm_runtime_disabled) {
		ret = bg9x_gps_send_cmd("AT+CPSMS=0", MDM_CMD_TIMEOUT);
		LOG_INF("%s: PSM disabled (ret=%d)", reason, ret);
	} else {
		ret = bg9x_psm_encode_tau(psm_runtime_wake_seconds, tau, &applied_seconds);
		if (ret < 0) {
			LOG_WRN("%s: PSM TAU encode failed for %us: %d, falling back to config",
				reason, psm_runtime_wake_seconds, ret);
			strncpy(tau, CONFIG_MODEM_QUECTEL_BG9X_PSM_TAU, sizeof(tau));
			tau[sizeof(tau) - 1] = '\0';
			applied_seconds = 0U;
		}

		snprintk(cmd, sizeof(cmd), "AT+CPSMS=1,,,\"%s\",\"%s\"",
			 tau, CONFIG_MODEM_QUECTEL_BG9X_PSM_ACTIVE_TIME);
		ret = bg9x_gps_send_cmd(cmd, MDM_CMD_TIMEOUT);
		LOG_INF("%s: PSM enabled wake=%us tau=%s applied=%us ret=%d",
			reason,
			psm_runtime_wake_seconds,
			tau,
			applied_seconds,
			ret);
	}

	if (ret == 0) {
		psm_policy_dirty = false;
	}

	return ret;
}

int bg9x_psm_set(bool enable)
{
	return bg9x_psm_set_interval(enable, psm_runtime_wake_seconds);
}

int bg9x_psm_set_interval(bool enable, uint32_t wake_seconds)
{
	int ret;
	bool status_high = true;

	if (enable && wake_seconds > 0U) {
		psm_runtime_wake_seconds = wake_seconds;
	}

	psm_policy_dirty = true;

	if (!enable) {
		psm_runtime_disabled = true;
		modem_ap_ready_set(true, "psm-disable");
	} else {
		psm_runtime_disabled = false;
		cycle_begin_prepared = false;
	}

	/* Allow the app to latch PSM preference before the modem is fully set up.
	 * Normal setup/runtime-restore paths will apply the CPSMS command later.
	 */
	if (!mdata.setup_complete) {
		LOG_INF("PSM preference latched enable=%d until modem setup completes",
			enable ? 1 : 0);
		return 0;
	}

	/* Apply the CPSMS setting while the modem is still awake.
	 * If the modem is already asleep at pre-sleep time, do not wake it back up
	 * just to send another CPSMS command. Latch the desired policy and apply it
	 * on the next active wake/setup path instead.
	 */
	ret = bg9x_apply_psm_preference("runtime request");
	if (enable) {
		modem_ap_ready_set(false, "psm-enable");
	}
	if (ret == 0) {
		return 0;
	}

#if DT_INST_NODE_HAS_PROP(0, mdm_status_gpios)
	status_high = modem_runtime_line_asserted(&status_gpio);
#endif

	if (enable && !status_high) {
		LOG_WRN("PSM request deferred: modem already asleep/status low (ret=%d)", ret);
		return 0;
	}

	LOG_WRN("PSM direct request failed (%d); preparing runtime wake", ret);
	mdata.runtime_wake_pending = true;
	ret = modem_ensure_ready();
	if (ret < 0) {
		LOG_WRN("PSM set: modem wake failed: %d", ret);
		return ret;
	}

	return 0;
}

int bg9x_cycle_begin(void)
{
	bool wake_needed = false;
	bool status_asserted = true;
	bool psm_asserted = false;
	int ret;

	if (mdata.gps_active) {
		return 0;
	}

	modem_ap_ready_set(true, "cycle-begin");
	k_sleep(MDM_RUNTIME_AP_READY_SETTLE);

	if (!mdata.setup_complete) {
		LOG_WRN("Cycle begin with incomplete modem setup");
		return modem_setup();
	}

#if DT_INST_NODE_HAS_PROP(0, mdm_status_gpios)
	status_asserted = modem_runtime_line_asserted(&status_gpio);
	if (!status_asserted) {
		wake_needed = true;
	}
#endif
#if DT_INST_NODE_HAS_PROP(0, mdm_psm_gpios)
	psm_asserted = modem_runtime_line_asserted(&psm_gpio);
	if (psm_asserted) {
		wake_needed = true;
	}
#endif

	if (wake_needed) {
		if (status_asserted && psm_asserted) {
			LOG_INF("Cycle begin sees PSM asserted with STATUS high; trying direct AT probe");
			k_sleep(K_MSEC(250));
			ret = modem_runtime_at_ping(K_SECONDS(3));
			if (ret == 0) {
				cycle_begin_prepared = true;
				LOG_INF("Cycle begin AT ready from asserted STATUS");
				return 0;
			}
			LOG_WRN("Cycle begin direct AT probe failed: %d", ret);
		}

		LOG_INF("Cycle begin detected sleeping modem");
		modem_log_runtime_lines_info("cycle begin");
		ret = modem_runtime_prepare();
		if (ret == 0) {
			cycle_begin_prepared = true;
			LOG_INF("Cycle begin runtime wake prepared");
		}
		return ret;
	}

	ret = modem_ensure_ready();
	if (ret < 0) {
		return ret;
	}

	ret = modem_runtime_at_ping(K_SECONDS(2));
	if (ret == 0) {
		cycle_begin_prepared = true;
		LOG_INF("Cycle begin AT ready");
		return 0;
	}

	LOG_WRN("Cycle begin AT probe failed after wake prep: %d", ret);
	mdata.runtime_wake_pending = true;
	cycle_begin_prepared = false;
	ret = modem_runtime_prepare();
	if (ret == 0) {
		cycle_begin_prepared = true;
		LOG_INF("Cycle begin runtime wake prepared");
	}
	return ret;
}

int bg9x_gps_xtra_policy_set(bool enabled, uint8_t retention_days, uint8_t mode)
{
	if (retention_days != 1U && retention_days != 2U &&
	    retention_days != 3U && retention_days != 7U) {
		return -EINVAL;
	}

	if (mode > 2U) {
		return -EINVAL;
	}

	if (xtra_policy_enabled == enabled &&
	    xtra_policy_retention_days == retention_days &&
	    xtra_policy_mode == mode) {
		return 0;
	}

	xtra_policy_enabled = enabled;
	xtra_policy_retention_days = retention_days;
	xtra_policy_mode = mode;
	xtra_policy_dirty = true;
	return 0;
}

int bg9x_gps_session_begin(void)
{
	int ret;

	if (!mdata.setup_complete) {
		return -EHOSTDOWN;
	}

	if (mdata.gps_active) {
		return 0;
	}

	LOG_INF("GPS scaffold begin: switching modem from cellular to GNSS");

	/* Start the active cycle deliberately before the GNSS handoff so
	 * later AT commands don't discover sleep reactively. If the app
	 * already prepared this active cycle, skip the duplicate wake pass.
	 */
	if (!cycle_begin_prepared) {
		ret = bg9x_cycle_begin();
		if (ret < 0) {
			LOG_WRN("GPS scaffold cycle-begin wake failed: %d", ret);
			return ret;
		}
	}

	ret = bg9x_gps_prepare_command_path();
	if (ret < 0) {
		LOG_WRN("GPS scaffold command path not ready: %d", ret);
		return ret;
	}

	/* Keep the modem awake through GPS restore and the subsequent
	 * DNS/connect/send path. Re-enable PSM only when the app is about to
	 * return to sleep.
	 */
	if (!psm_runtime_disabled) {
		ret = bg9x_gps_send_cmd("AT+CPSMS=0", MDM_CMD_TIMEOUT);
		if (ret < 0) {
			LOG_WRN("GPS scaffold temporary PSM disable failed: %d; continuing active cycle",
				ret);
			psm_resume_after_gps = true;
		} else {
			psm_resume_after_gps = true;
			LOG_INF("GPS scaffold: temporarily disabled PSM for active cycle");
		}
	} else {
		psm_resume_after_gps = false;
	}

	ret = bg9x_gps_send_cmd("AT+CFUN=0", MDM_CMD_TIMEOUT);
	if (ret < 0) {
		LOG_WRN("GPS scaffold CFUN=0 failed: %d", ret);
		return ret;
	}

	k_sleep(K_SECONDS(2));

	if (!gps_static_cfg_done) {
		/* One-time GNSS policy setup. Re-sending these every cycle adds
		 * avoidable latency and was the immediate cause of later-cycle
		 * CFUN=0 timeouts during overnight soak.
		 */
		ret = bg9x_gps_send_cmd("AT+QGPSCFG=\"gnssconfig\",3",
					MDM_CMD_TIMEOUT);
		LOG_INF("GPS gnssconfig=3 ret=%d", ret);
		if (ret < 0) {
			LOG_WRN("GPS gnssconfig setup failed: %d", ret);
		} else {
			gps_static_cfg_done = true;
		}
	}

	ret = bg9x_apply_xtra_policy();
	if (ret < 0) {
		LOG_WRN("GPS XTRA policy apply failed: %d", ret);
	}

	ret = bg9x_gps_send_cmd("AT+QGPS=1", MDM_CMD_TIMEOUT);
	if (ret < 0) {
		LOG_WRN("GPS scaffold QGPS=1 failed: %d", ret);
		bg9x_gps_send_cmd("AT+CFUN=1", MDM_CMD_TIMEOUT);
		return ret;
	}

	if (gps_static_cfg_done) {
		/* Enable NMEA sentence query via AT+QGPSGNMEA once GNSS is live. */
		static bool gps_nmea_cfg_done;
		if (!gps_nmea_cfg_done) {
			ret = bg9x_gps_send_cmd("AT+QGPSCFG=\"nmeasrc\",1",
						MDM_CMD_TIMEOUT);
			if (ret < 0) {
				LOG_WRN("GPS nmeasrc setup failed: %d", ret);
			} else {
				gps_nmea_cfg_done = true;
			}
		}
	}

	mdata.gps_active = true;
	mdata.setup_complete = false;
	mdata.pdp_active = false;
	mdata.runtime_wake_pending = false;

	return 0;
}

int bg9x_gps_session_end(void)
{
	int ret;
	int counter;

	if (!mdata.gps_active) {
		return 0;
	}

	LOG_INF("GPS scaffold end: stopping GNSS and restoring cellular");

	ret = bg9x_gps_send_cmd("AT+QGPSEND", MDM_CMD_TIMEOUT);
	if (ret < 0) {
		LOG_WRN("GPS scaffold QGPSEND failed: %d", ret);
	}

	ret = bg9x_gps_send_cmd("AT+CFUN=1", MDM_CMD_TIMEOUT);
	if (ret < 0) {
		LOG_WRN("GPS scaffold CFUN=1 restore failed: %d", ret);
		return ret;
	}

	k_sleep(K_SECONDS(2));

	mdata.gps_active = false;
	mdata.runtime_wake_pending = false;
	cycle_begin_prepared = true;

	/* Wait for radio to re-register (no power cycle needed) */
	LOG_INF("GPS restore: waiting for cellular re-registration");
	k_sleep(K_SECONDS(5));

	/* Run lightweight setup (APN, echo off, etc.) */
	bg9x_gps_send_cmd("ATE0", MDM_CMD_TIMEOUT);
	bg9x_gps_send_cmd("ATH", MDM_CMD_TIMEOUT);
	bg9x_gps_send_cmd("AT+CMEE=1", MDM_CMD_TIMEOUT);
	bg9x_gps_send_cmd("AT+QICSGP=1,1,\"" MDM_APN "\",\"\",\"\",1", MDM_CMD_TIMEOUT);
	bg9x_gps_send_cmd("AT+QCFG=\"psm/urc\",1", MDM_CMD_TIMEOUT);

	/* Poll RSSI until valid */
	counter = 0;
	modem_rssi_query_work(NULL);
	k_sleep(MDM_WAIT_FOR_RSSI_DELAY);
	while (counter++ < MDM_WAIT_FOR_RSSI_COUNT &&
	       (mdata.mdm_rssi >= 0 || mdata.mdm_rssi <= -1000)) {
		modem_rssi_query_work(NULL);
		k_sleep(MDM_WAIT_FOR_RSSI_DELAY);
	}

	if (mdata.mdm_rssi >= 0 || mdata.mdm_rssi <= -1000) {
		LOG_WRN("GPS restore: RSSI invalid after polling, falling back to full setup");
		return modem_setup();
	}

	LOG_INF("GPS restore: RSSI=%d, activating PDP context", mdata.mdm_rssi);

	ret = modem_pdp_context_activate();
	if (ret < 0) {
		LOG_WRN("GPS restore: PDP activation failed: %d, falling back to full setup", ret);
		return modem_setup();
	}

	mdata.setup_complete = true;
	mdata.pdp_active = true;
	if (psm_resume_after_gps) {
		LOG_INF("GPS restore: deferring PSM re-enable until pre-sleep");
	}

	LOG_INF("GPS restore: cellular fully restored");
	return 0;
}

int bg9x_gps_get_fix(struct bg9x_gps_fix *fix, int timeout_s)
{
	struct modem_cmd qgpsloc_cmd[] = {
		MODEM_CMD_ARGS_MAX("+QGPSLOC: ", on_cmd_gps_qgpsloc, 0U, 16U, ","),
	};
	int64_t start_ms;
	int64_t elapsed_ms;
	int ret;

	if (fix == NULL) {
		return -EINVAL;
	}

	memset(fix, 0, sizeof(*fix));

	if (!mdata.gps_active) {
		return -EHOSTDOWN;
	}

	start_ms = k_uptime_get();
	gps_fix_received = false;
	memset(&gps_pending_fix, 0, sizeof(gps_pending_fix));

	LOG_INF("GPS polling for fix (timeout=%ds)", timeout_s);

	/* Poll QGPSLOC every 2 seconds until fix or timeout */
	while (true) {
		elapsed_ms = k_uptime_get() - start_ms;
		if (elapsed_ms >= (int64_t)timeout_s * 1000) {
			LOG_WRN("GPS fix timeout after %ds", timeout_s);
			return -ETIMEDOUT;
		}

		ret = modem_cmd_send(&mctx.iface, &mctx.cmd_handler,
				     qgpsloc_cmd, ARRAY_SIZE(qgpsloc_cmd),
				     "AT+QGPSLOC=0",
				     &mdata.sem_response, MDM_CMD_TIMEOUT);

		if (ret == 0 && gps_fix_received && gps_pending_fix.valid) {
			break;
		}

		/* +CME ERROR: 516 = no fix yet, keep polling */
		LOG_INF("GPS no fix yet (ret=%d elapsed=%llds)", ret, elapsed_ms / 1000);
		k_sleep(K_SECONDS(2));
	}

	/* Record fix time */
	gps_pending_fix.fix_time_ms = (uint32_t)(k_uptime_get() - start_ms);

	LOG_INF("GPS fix acquired in %ums: lat=%.6f lon=%.6f alt=%.1f sats=%u",
		gps_pending_fix.fix_time_ms, gps_pending_fix.latitude,
		gps_pending_fix.longitude, (double)gps_pending_fix.altitude,
		gps_pending_fix.sats_in_use);

	/* Query supplemental NMEA sentences (instant, GPS is still active) */

	/* GSA for PDOP/VDOP */
	ret = bg9x_gps_query_nmea("GSA");
	if (ret == 0 && gps_nmea_buf[0] != '\0') {
		gps_parse_gsa(gps_nmea_buf);
		LOG_DBG("GSA parsed: pdop=%.1f vdop=%.1f",
			(double)gps_pending_fix.pdop, (double)gps_pending_fix.vdop);
	}

	/* GSV for sats_in_view and per-sat detail */
	ret = bg9x_gps_query_nmea("GSV");
	if (ret == 0 && gps_nmea_buf[0] != '\0') {
		gps_parse_gsv(gps_nmea_buf);
		LOG_DBG("GSV parsed: sats_in_view=%u sat_count=%u",
			gps_pending_fix.sats_in_view, gps_pending_fix.sat_count);
	}

	/* VTG for magnetic course */
	ret = bg9x_gps_query_nmea("VTG");
	if (ret == 0 && gps_nmea_buf[0] != '\0') {
		gps_parse_vtg(gps_nmea_buf);
	}

	/* RMC for magnetic variation + raw sentence */
	ret = bg9x_gps_query_nmea("RMC");
	if (ret == 0 && gps_nmea_buf[0] != '\0') {
		gps_parse_rmc(gps_nmea_buf);
	}

	/* GGA raw sentence */
	ret = bg9x_gps_query_nmea("GGA");
	if (ret == 0 && gps_nmea_buf[0] != '\0') {
		strncpy(gps_pending_fix.nmea_gga, gps_nmea_buf,
			sizeof(gps_pending_fix.nmea_gga) - 1);
		gps_pending_fix.nmea_gga[sizeof(gps_pending_fix.nmea_gga) - 1] = '\0';
	}

	memcpy(fix, &gps_pending_fix, sizeof(*fix));
	return 0;
}

static int modem_pdp_context_verify(void)
{
	struct modem_cmd cmd[] = {
		MODEM_CMD("+QIACT: ", on_cmd_atcmdinfo_qiact, 2U, ","),
	};
	int ret;

	mdata.pdp_active = false;
	ret = modem_cmd_send(&mctx.iface, &mctx.cmd_handler,
			     cmd, ARRAY_SIZE(cmd), "AT+QIACT?",
			     &mdata.sem_response, MDM_CMD_TIMEOUT);
	if (ret < 0) {
		LOG_WRN("PDP verify command failed: %d", ret);
		return ret;
	}

	if (!mdata.pdp_active) {
		LOG_WRN("PDP context 1 not active after QIACT");
		return -ENETDOWN;
	}

	return 0;
}

static int modem_optional_gpio_get(const struct gpio_dt_spec *gpio)
{
	int ret;

	if (gpio->port == NULL) {
		return -ENOTSUP;
	}

	ret = gpio_pin_get_dt(gpio);
	if (ret < 0) {
		LOG_WRN("Failed reading GPIO pin %d ret=%d", gpio->pin, ret);
		return ret;
	}

	return ret;
}

static const char *modem_gpio_state_to_str(int value)
{
	switch (value) {
	case 0:
		return "0";
	case 1:
		return "1";
	case -ENOTSUP:
		return "na";
	default:
		return "err";
	}
}

static bool modem_runtime_line_asserted(const struct gpio_dt_spec *gpio)
{
	return modem_optional_gpio_get(gpio) > 0;
}

static void modem_log_runtime_lines_dbg(const char *label)
{
	int status = -ENOTSUP;
	int ap_ready = -ENOTSUP;
	int psm = -ENOTSUP;
	int ri = -ENOTSUP;

#if DT_INST_NODE_HAS_PROP(0, mdm_status_gpios)
	status = modem_optional_gpio_get(&status_gpio);
#endif
#if DT_INST_NODE_HAS_PROP(0, mdm_ap_ready_gpios)
	ap_ready = modem_optional_gpio_get(&ap_ready_gpio);
#endif
#if DT_INST_NODE_HAS_PROP(0, mdm_psm_gpios)
	psm = modem_optional_gpio_get(&psm_gpio);
#endif
#if DT_INST_NODE_HAS_PROP(0, mdm_ri_gpios)
	ri = modem_optional_gpio_get(&ri_gpio);
#endif

	LOG_DBG("%s modem lines: STATUS=%s AP_READY=%s PSM=%s RI=%s",
		label,
		modem_gpio_state_to_str(status),
		modem_gpio_state_to_str(ap_ready),
		modem_gpio_state_to_str(psm),
		modem_gpio_state_to_str(ri));
}

static void modem_log_runtime_lines_info(const char *label)
{
	int status = -ENOTSUP;
	int ap_ready = -ENOTSUP;
	int psm = -ENOTSUP;
	int ri = -ENOTSUP;

#if DT_INST_NODE_HAS_PROP(0, mdm_status_gpios)
	status = modem_optional_gpio_get(&status_gpio);
#endif
#if DT_INST_NODE_HAS_PROP(0, mdm_ap_ready_gpios)
	ap_ready = modem_optional_gpio_get(&ap_ready_gpio);
#endif
#if DT_INST_NODE_HAS_PROP(0, mdm_psm_gpios)
	psm = modem_optional_gpio_get(&psm_gpio);
#endif
#if DT_INST_NODE_HAS_PROP(0, mdm_ri_gpios)
	ri = modem_optional_gpio_get(&ri_gpio);
#endif

	LOG_INF("%s modem lines: STATUS=%s AP_READY=%s PSM=%s RI=%s",
		label,
		modem_gpio_state_to_str(status),
		modem_gpio_state_to_str(ap_ready),
		modem_gpio_state_to_str(psm),
		modem_gpio_state_to_str(ri));
}

int bg9x_runtime_snapshot_get(struct bg9x_runtime_snapshot *snapshot, bool probe_at)
{
	uint8_t allocated = 0U;
	uint8_t connected = 0U;

	if (snapshot == NULL) {
		return -EINVAL;
	}

	memset(snapshot, 0, sizeof(*snapshot));
	snapshot->setup_complete = mdata.setup_complete;
	snapshot->runtime_wake_pending = mdata.runtime_wake_pending;
	snapshot->dns_query_active = mdata.dns_query_active;
	snapshot->pdp_active = mdata.pdp_active;
	snapshot->gps_active = mdata.gps_active;
	snapshot->mdm_rssi = mdata.mdm_rssi;
	snapshot->at_probe_ret = -ENODATA;

#if DT_INST_NODE_HAS_PROP(0, mdm_status_gpios)
	snapshot->status_gpio = modem_optional_gpio_get(&status_gpio);
#else
	snapshot->status_gpio = -ENOTSUP;
#endif
#if DT_INST_NODE_HAS_PROP(0, mdm_ap_ready_gpios)
	snapshot->ap_ready_gpio = modem_optional_gpio_get(&ap_ready_gpio);
#else
	snapshot->ap_ready_gpio = -ENOTSUP;
#endif
#if DT_INST_NODE_HAS_PROP(0, mdm_psm_gpios)
	snapshot->psm_gpio = modem_optional_gpio_get(&psm_gpio);
#else
	snapshot->psm_gpio = -ENOTSUP;
#endif
#if DT_INST_NODE_HAS_PROP(0, mdm_ri_gpios)
	snapshot->ri_gpio = modem_optional_gpio_get(&ri_gpio);
#else
	snapshot->ri_gpio = -ENOTSUP;
#endif

	for (size_t i = 0U; i < ARRAY_SIZE(mdata.sockets); i++) {
		if (modem_socket_is_allocated(&mdata.socket_config, &mdata.sockets[i])) {
			allocated++;
			if (mdata.sockets[i].is_connected) {
				connected++;
			}
		}
	}

	snapshot->allocated_sockets = allocated;
	snapshot->connected_sockets = connected;

	if (!probe_at) {
		return 0;
	}

	snapshot->at_probe_ret = modem_runtime_at_ping(K_MSEC(500));
	snapshot->at_ready = (snapshot->at_probe_ret == 0);
	return 0;
}

int bg9x_connected_socket_qistate_get(bool prepare_on_timeout, int *sock_id, bool *present,
				      char *resp, size_t resp_size)
{
	char cmd[24];
	int found_sock_id = -1;
	int ret;

	if (sock_id != NULL) {
		*sock_id = -1;
	}
	if (present != NULL) {
		*present = false;
	}
	if (resp != NULL && resp_size > 0U) {
		resp[0] = '\0';
	}

	for (size_t i = 0U; i < ARRAY_SIZE(mdata.sockets); i++) {
		if (modem_socket_is_allocated(&mdata.socket_config, &mdata.sockets[i]) &&
		    mdata.sockets[i].is_connected) {
			found_sock_id = mdata.sockets[i].id;
			break;
		}
	}

	if (found_sock_id < 0) {
		return -ENOENT;
	}

	if (sock_id != NULL) {
		*sock_id = found_sock_id;
	}

	snprintk(cmd, sizeof(cmd), "AT+QISTATE=1,%d", found_sock_id);
	ret = bg9x_at_cmd_passthrough_internal(cmd, resp, resp_size, prepare_on_timeout);
	if (ret < 0) {
		return ret;
	}

	if (present != NULL) {
		*present = bg9x_qistate_has_socket(resp, found_sock_id);
	}

	return 0;
}

int bg9x_connected_socket_reopen(void)
{
	for (size_t i = 0U; i < ARRAY_SIZE(mdata.sockets); i++) {
		if (modem_socket_is_allocated(&mdata.socket_config, &mdata.sockets[i]) &&
		    mdata.sockets[i].is_connected) {
			return bg9x_recover_vanished_socket(&mdata.sockets[i]);
		}
	}

	return -ENOENT;
}

int bg9x_connected_socket_resume_prepare(bool *present, bool *reopened,
					 char *resp, size_t resp_size)
{
	int found_sock_id = -1;
	int ret;
	bool socket_present = false;

	if (present != NULL) {
		*present = false;
	}
	if (reopened != NULL) {
		*reopened = false;
	}
	if (resp != NULL && resp_size > 0U) {
		resp[0] = '\0';
	}

	for (size_t i = 0U; i < ARRAY_SIZE(mdata.sockets); i++) {
		if (modem_socket_is_allocated(&mdata.socket_config, &mdata.sockets[i]) &&
		    mdata.sockets[i].is_connected) {
			found_sock_id = (int)i;
			break;
		}
	}

	if (found_sock_id < 0) {
		return -ENOENT;
	}

	ret = modem_runtime_prepare();
	if (ret < 0) {
		return ret;
	}

	ret = bg9x_connected_socket_qistate_get(false, NULL, &socket_present, resp, resp_size);
	if (ret < 0) {
		return ret;
	}

	if (present != NULL) {
		*present = socket_present;
	}

	if (socket_present) {
		return 0;
	}

	ret = bg9x_recover_vanished_socket(&mdata.sockets[found_sock_id]);
	if (ret < 0) {
		return ret;
	}

	if (reopened != NULL) {
		*reopened = true;
	}

	if (resp != NULL && resp_size > 0U) {
		resp[0] = '\0';
	}
	(void)bg9x_connected_socket_qistate_get(false, NULL, present, resp, resp_size);
	return 0;
}

int bg9x_runtime_prepare(void)
{
	return modem_runtime_prepare();
}

static int modem_wait_for_runtime_lines(k_timeout_t timeout)
{
	int64_t deadline = k_uptime_get() + k_ticks_to_ms_floor64(timeout.ticks);
	bool have_status = false;
	bool have_psm = false;

#if DT_INST_NODE_HAS_PROP(0, mdm_status_gpios)
	have_status = true;
#endif
#if DT_INST_NODE_HAS_PROP(0, mdm_psm_gpios)
	have_psm = true;
#endif

	if (!have_status && !have_psm) {
		k_sleep(K_SECONDS(2));
		return 0;
	}

	while (k_uptime_get() <= deadline) {
		bool status_ready = true;
		bool psm_ready = true;

#if DT_INST_NODE_HAS_PROP(0, mdm_status_gpios)
		status_ready = modem_runtime_line_asserted(&status_gpio);
#endif
#if DT_INST_NODE_HAS_PROP(0, mdm_psm_gpios)
		psm_ready = !modem_runtime_line_asserted(&psm_gpio);
#endif
		if (status_ready && psm_ready) {
			return 0;
		}

		k_sleep(K_MSEC(100));
	}

	modem_log_runtime_lines_info("timeout waiting for awake lines");
	return -ETIMEDOUT;
}

/* Func: send_socket_data
 * Desc: This function will send "binary" data over the socket object.
 */
static ssize_t send_socket_data(struct modem_socket *sock,
				const struct sockaddr *dst_addr,
				struct modem_cmd *handler_cmds,
				size_t handler_cmds_len,
				const char *buf, size_t buf_len,
				k_timeout_t timeout)
{
	int  ret;
	char send_buf[sizeof("AT+QISEND=##,####")] = {0};
	char ctrlz = 0x1A;

	if (buf_len > MDM_MAX_DATA_LENGTH) {
		buf_len = MDM_MAX_DATA_LENGTH;
	}

	/* Create a buffer with the correct params. */
	mdata.sock_written = buf_len;
	snprintk(send_buf, sizeof(send_buf), "AT+QISEND=%d,%ld", sock->id, (long) buf_len);

	/* Setup the locks correctly. */
	(void)k_sem_take(&mdata.cmd_handler_data.sem_tx_lock, K_FOREVER);
	k_sem_reset(&mdata.sem_tx_ready);

	/* Send the Modem command. */
	ret = modem_cmd_send_nolock(&mctx.iface, &mctx.cmd_handler,
				    NULL, 0U, send_buf, NULL, K_NO_WAIT);
	if (ret < 0) {
		goto exit;
	}

	/* set command handlers */
	ret = modem_cmd_handler_update_cmds(&mdata.cmd_handler_data,
					    handler_cmds, handler_cmds_len,
					    true);
	if (ret < 0) {
		goto exit;
	}

	/* Wait for '>' */
	ret = k_sem_take(&mdata.sem_tx_ready, K_MSEC(5000));
	if (ret < 0) {
		/* Didn't get the data prompt - Exit. */
		LOG_DBG("Timeout waiting for tx");
		goto exit;
	}

	/* Write all data on the console and send CTRL+Z. */
	modem_cmd_send_data_nolock(&mctx.iface, buf, buf_len);
	modem_cmd_send_data_nolock(&mctx.iface, &ctrlz, 1);

	/* Wait for 'SEND OK' or 'SEND FAIL' */
	k_sem_reset(&mdata.sem_response);
	ret = k_sem_take(&mdata.sem_response, timeout);
	if (ret < 0) {
		LOG_DBG("No send response");
		goto exit;
	}

	ret = modem_cmd_handler_get_error(&mdata.cmd_handler_data);
	if (ret != 0) {
		LOG_DBG("Failed to send data");
	}

exit:
	/* unset handler commands and ignore any errors */
	(void)modem_cmd_handler_update_cmds(&mdata.cmd_handler_data,
					    NULL, 0U, false);
	k_sem_give(&mdata.cmd_handler_data.sem_tx_lock);

	if (ret < 0) {
		return ret;
	}

	/* Return the amount of data written on the socket. */
	return mdata.sock_written;
}

static socklen_t bg9x_sockaddr_len(const struct sockaddr *addr)
{
	if (addr == NULL) {
		return 0;
	}

	switch (addr->sa_family) {
	case AF_INET:
		return sizeof(struct sockaddr_in);
	case AF_INET6:
		return sizeof(struct sockaddr_in6);
	default:
		return sizeof(struct sockaddr);
	}
}

static bool bg9x_qistate_has_socket(const char *resp, int sock_id)
{
	char needle[16];

	if (resp == NULL || resp[0] == '\0') {
		return false;
	}

	snprintk(needle, sizeof(needle), "+QISTATE: %d,", sock_id);
	return strstr(resp, needle) != NULL;
}

static int bg9x_recover_vanished_socket(struct modem_socket *sock)
{
	socklen_t addrlen;

	if (sock == NULL) {
		return -EINVAL;
	}

	addrlen = bg9x_sockaddr_len((const struct sockaddr *)&sock->dst);
	if (addrlen == 0) {
		return -EINVAL;
	}

	LOG_WRN("Recovering vanished modem socket id=%d fd=%d", sock->id, sock->sock_fd);
	(void)socket_close_cmd(sock);
	return offload_connect(sock, (const struct sockaddr *)&sock->dst, addrlen);
}

/* Func: offload_sendto
 * Desc: This function will send data on the socket object.
 */
static ssize_t offload_sendto(void *obj, const void *buf, size_t len,
			      int flags, const struct sockaddr *to,
			      socklen_t tolen)
{
	int ret;
	bool socket_present = true;
	char qistate_cmd[24];
	char qistate_resp[192] = { 0 };
	struct modem_socket *sock = (struct modem_socket *) obj;

	/* Here's how sending data works,
	 * -> We firstly send the "AT+QISEND" command on the given socket and
	 *    specify the length of data to be transferred.
	 * -> In response to "AT+QISEND" command, the modem may respond with a
	 *    data prompt (>) or not respond at all. If it doesn't respond, we
	 *    exit. If it does respond with a data prompt (>), we move forward.
	 * -> We plainly write all data on the UART and terminate by sending a
	 *    CTRL+Z. Once the modem receives CTRL+Z, it starts processing the
	 *    data and will respond with either "SEND OK", "SEND FAIL" or "ERROR".
	 *    Here we are registering handlers for the first two responses. We
	 *    already have a handler for the "generic" error response.
	 */
	struct modem_cmd cmd[] = {
		MODEM_CMD_DIRECT(">", on_cmd_tx_ready),
		MODEM_CMD("SEND OK", on_cmd_send_ok,   0, ","),
		MODEM_CMD("SEND FAIL", on_cmd_send_fail, 0, ","),
	};

	/* Ensure that valid parameters are passed. */
	if (!buf || len == 0) {
		errno = EINVAL;
		return -1;
	}

	if (!sock->is_connected) {
		errno = ENOTCONN;
		return -1;
	}

	/*
	 * Zephyr's DTLS layer uses sendto() with the peer address even after the
	 * inner UDP socket has been connected. The BG95 socket is already bound to
	 * a single remote peer by QIOPEN, so the destination argument is redundant
	 * here and can be ignored.
	 */

	ret = send_socket_data(sock, to, cmd, ARRAY_SIZE(cmd), buf, len,
			       MDM_CMD_TIMEOUT);
	if (ret < 0) {
		snprintk(qistate_cmd, sizeof(qistate_cmd), "AT+QISTATE=1,%d", sock->id);
		LOG_WRN("offload_sendto failed ret=%d sock=%d connected=%d pdp=%d setup=%d wake_pending=%d",
			ret, sock->id, sock->is_connected ? 1 : 0,
			mdata.pdp_active ? 1 : 0, mdata.setup_complete ? 1 : 0,
			mdata.runtime_wake_pending ? 1 : 0);
		if (bg9x_at_cmd_passthrough(qistate_cmd, qistate_resp, sizeof(qistate_resp)) < 0) {
			LOG_WRN("offload_sendto qistate query failed sock=%d", sock->id);
		} else {
			LOG_WRN("offload_sendto qistate sock=%d resp=%s", sock->id, qistate_resp);
			socket_present = bg9x_qistate_has_socket(qistate_resp, sock->id);
		}

		if (!socket_present && mdata.pdp_active && mdata.setup_complete) {
			ret = bg9x_recover_vanished_socket(sock);
			if (ret == 0) {
				LOG_INF("Recovered vanished modem socket id=%d; retrying send", sock->id);
				ret = send_socket_data(sock, to, cmd, ARRAY_SIZE(cmd), buf, len,
						       MDM_CMD_TIMEOUT);
				if (ret >= 0) {
					errno = 0;
					return ret;
				}
				LOG_WRN("Recovered modem socket send retry failed ret=%d sock=%d",
					ret, sock->id);
			} else {
				LOG_WRN("Recovering vanished modem socket failed ret=%d sock=%d",
					ret, sock->id);
			}
		}
		errno = -ret;
		return -1;
	}

	/* Data was written successfully. */
	errno = 0;
	return ret;
}

/* Func: offload_recvfrom
 * Desc: This function will receive data on the socket object.
 */
static ssize_t offload_recvfrom(void *obj, void *buf, size_t len,
				int flags, struct sockaddr *from,
				socklen_t *fromlen)
{
	struct modem_socket *sock = (struct modem_socket *)obj;
	char   sendbuf[sizeof("AT+QIRD=##,#####")] = {0};
	int    ret;
	int    next_packet_size;
	size_t read_len;
	struct socket_read_data sock_data;

	/* Modem command to read the data. */
	struct modem_cmd data_cmd[] = { MODEM_CMD("+QIRD: ", on_cmd_sock_readdata, 0U, "") };

	if (!buf || len == 0) {
		errno = EINVAL;
		return -1;
	}

	if (flags & ZSOCK_MSG_PEEK) {
		errno = ENOTSUP;
		return -1;
	}

	read_len = MIN(len, (size_t)MDM_MAX_DATA_LENGTH);

	next_packet_size = modem_socket_next_packet_size(&mdata.socket_config, sock);
	if (!next_packet_size) {
		if (flags & ZSOCK_MSG_DONTWAIT) {
			errno = EAGAIN;
			return -1;
		}

		modem_socket_wait_data(&mdata.socket_config, sock);
		next_packet_size = modem_socket_next_packet_size(&mdata.socket_config, sock);
		if (!next_packet_size) {
			errno = EAGAIN;
			return -1;
		}
	}

	snprintk(sendbuf, sizeof(sendbuf), "AT+QIRD=%d,%zd", sock->id, read_len);

	/* Socket read settings */
	(void) memset(&sock_data, 0, sizeof(sock_data));
	sock_data.recv_buf     = buf;
	sock_data.recv_buf_len = read_len;
	sock_data.recv_addr    = from;
	sock->data	       = &sock_data;
	mdata.sock_fd	       = sock->sock_fd;

	/* Tell the modem to give us data (AT+QIRD=id,data_len). */
	ret = modem_cmd_send(&mctx.iface, &mctx.cmd_handler,
			     data_cmd, ARRAY_SIZE(data_cmd), sendbuf, &mdata.sem_response,
			     MDM_CMD_TIMEOUT);
	if (ret < 0) {
		errno = -ret;
		ret = -1;
		goto exit;
	}

	/* HACK: use dst address as from */
	if (from && fromlen) {
		*fromlen = sizeof(sock->dst);
		memcpy(from, &sock->dst, *fromlen);
	}

	/* return length of received data */
	errno = 0;
	ret = sock_data.recv_read_len;

exit:
	/* clear socket data */
	sock->data = NULL;
	return ret;
}

/* Func: offload_read
 * Desc: This function reads data from the given socket object.
 */
static ssize_t offload_read(void *obj, void *buffer, size_t count)
{
	return offload_recvfrom(obj, buffer, count, 0, NULL, 0);
}

/* Func: offload_write
 * Desc: This function writes data to the given socket object.
 */
static ssize_t offload_write(void *obj, const void *buffer, size_t count)
{
	return offload_sendto(obj, buffer, count, 0, NULL, 0);
}

static int offload_copy_int_opt(void *optval, socklen_t *optlen, int value)
{
	if (optval == NULL || optlen == NULL || *optlen < sizeof(int)) {
		return -EINVAL;
	}

	*(int *)optval = value;
	*optlen = sizeof(int);
	return 0;
}

static int offload_socket_index(const struct modem_socket *sock)
{
	if (sock == NULL) {
		return -EINVAL;
	}

	if (sock < &mdata.sockets[0] || sock >= &mdata.sockets[ARRAY_SIZE(mdata.sockets)]) {
		return -EINVAL;
	}

	return (int)(sock - &mdata.sockets[0]);
}

static int offload_socket_get_flags(const struct modem_socket *sock)
{
	int idx = offload_socket_index(sock);

	if (idx < 0) {
		return idx;
	}

	return mdata.socket_nonblock[idx] ? O_NONBLOCK : 0;
}

static int offload_socket_set_flags(const struct modem_socket *sock, int flags)
{
	int idx = offload_socket_index(sock);

	if (idx < 0) {
		return idx;
	}

	mdata.socket_nonblock[idx] = (flags & O_NONBLOCK) != 0;
	return 0;
}

static int offload_getsockopt(void *obj, int level, int optname,
			      void *optval, socklen_t *optlen)
{
	struct modem_socket *sock = (struct modem_socket *)obj;

	if (sock == NULL || optval == NULL || optlen == NULL) {
		return -EINVAL;
	}

	if (level != SOL_SOCKET) {
		return -ENOPROTOOPT;
	}

	switch (optname) {
	case SO_ERROR:
		return offload_copy_int_opt(optval, optlen, 0);
	case SO_TYPE:
		return offload_copy_int_opt(optval, optlen, sock->type);
	case SO_PROTOCOL:
		return offload_copy_int_opt(optval, optlen, sock->ip_proto);
	case SO_DOMAIN:
		return offload_copy_int_opt(optval, optlen, sock->family);
	default:
		return -ENOPROTOOPT;
	}
}

static int offload_setsockopt(void *obj, int level, int optname,
			      const void *optval, socklen_t optlen)
{
	ARG_UNUSED(obj);

	if (level != SOL_SOCKET) {
		return -ENOPROTOOPT;
	}

	if (optval == NULL) {
		return -EINVAL;
	}

	switch (optname) {
	case SO_RCVTIMEO:
	case SO_SNDTIMEO:
		if (optlen < sizeof(struct zsock_timeval)) {
			return -EINVAL;
		}
		return 0;
	default:
		return -ENOPROTOOPT;
	}
}

/* Func: offload_ioctl
 * Desc: Function call to handle various misc requests.
 */
static int offload_ioctl(void *obj, unsigned int request, va_list args)
{
	struct modem_socket *sock = (struct modem_socket *)obj;

	switch (request) {
	case F_GETFL: {
		int flags = offload_socket_get_flags(sock);

		if (flags < 0) {
			errno = -flags;
			return -1;
		}

		return flags;
	}
	case F_SETFL: {
		int flags = va_arg(args, int);
		int ret = offload_socket_set_flags(sock, flags);

		if (ret < 0) {
			errno = -ret;
			return -1;
		}

		return 0;
	}
	case ZFD_IOCTL_POLL_PREPARE: {
		struct zsock_pollfd *pfd;
		struct k_poll_event **pev;
		struct k_poll_event *pev_end;

		pfd = va_arg(args, struct zsock_pollfd *);
		pev = va_arg(args, struct k_poll_event **);
		pev_end = va_arg(args, struct k_poll_event *);

		return modem_socket_poll_prepare(&mdata.socket_config, obj, pfd, pev, pev_end);
	}
	case ZFD_IOCTL_POLL_UPDATE: {
		struct zsock_pollfd *pfd;
		struct k_poll_event **pev;

		pfd = va_arg(args, struct zsock_pollfd *);
		pev = va_arg(args, struct k_poll_event **);

		return modem_socket_poll_update(obj, pfd, pev);
	}
	case ZFD_IOCTL_FIONBIO: {
		int *nonblock = va_arg(args, int *);
		int ret;

		if (nonblock == NULL) {
			errno = EINVAL;
			return -1;
		}

		ret = offload_socket_set_flags(sock, *nonblock ? O_NONBLOCK : 0);
		if (ret < 0) {
			errno = -ret;
			return -1;
		}

		return 0;
	}

	default:
		errno = EINVAL;
		return -1;
	}
}

/* Func: offload_connect
 * Desc: This function will connect with a provided TCP.
 */
static int offload_connect(void *obj, const struct sockaddr *addr,
						   socklen_t addrlen)
{
	struct modem_socket *sock     = (struct modem_socket *) obj;
	uint16_t	    dst_port  = 0;
	const char	    *protocol = "TCP";
	struct modem_cmd    cmd[]     = { MODEM_CMD("+QIOPEN: ", on_cmd_atcmdinfo_sockopen, 2U, ",") };
	char		    buf[sizeof("AT+QIOPEN=#,#,'###','###',"
				       "####.####.####.####.####.####.####.####,######,"
				       "0,0")] = {0};
	int		    ret = 0;
	int		    connect_ret = -EIO;
	int		    attempt;
	char		    ip_str[NET_IPV6_ADDR_LEN];

	ret = modem_ensure_ready();
	if (ret < 0) {
		errno = -ret;
		return -1;
	}

	/* Verify socket has been allocated */
	if (modem_socket_is_allocated(&mdata.socket_config, sock) == false) {
		LOG_ERR("Invalid socket_id(%d) from fd:%d",
			sock->id, sock->sock_fd);
		errno = EINVAL;
		return -1;
	}

	if (sock->is_connected == true) {
		LOG_ERR("Socket is already connected!! socket_id(%d), socket_fd:%d",
			sock->id, sock->sock_fd);
		errno = EISCONN;
		return -1;
	}

	/* Find the correct destination port. */
	if (addr->sa_family == AF_INET6) {
		dst_port = ntohs(net_sin6(addr)->sin6_port);
	} else if (addr->sa_family == AF_INET) {
		dst_port = ntohs(net_sin(addr)->sin_port);
	}

	if (sock->type == SOCK_DGRAM || sock->ip_proto == IPPROTO_UDP) {
		protocol = "UDP";
	}

	k_sem_reset(&mdata.sem_sock_conn);

	ret = modem_context_sprint_ip_addr(addr, ip_str, sizeof(ip_str));
	if (ret != 0) {
		LOG_ERR("Error formatting IP string %d", ret);
		LOG_ERR("Closing the socket!!!");
		socket_close(sock);
		errno = -ret;
		return -1;
	}

	/* Formulate the complete string. */
	snprintk(buf, sizeof(buf), "AT+QIOPEN=%d,%d,\"%s\",\"%s\",%d,0,0", 1, sock->id, protocol,
		 ip_str, dst_port);

	for (attempt = 0; attempt < MDM_SOCKET_OPEN_ATTEMPTS; attempt++) {
		if (mdata.runtime_wake_pending || attempt > 0) {
			ret = modem_runtime_prepare();
			if (ret < 0) {
				errno = -ret;
				return -1;
			}
		}

		k_sem_reset(&mdata.sem_sock_conn);

		/* Send out the command. */
		ret = modem_cmd_send(&mctx.iface, &mctx.cmd_handler,
				     NULL, 0U, buf,
				     &mdata.sem_response, K_SECONDS(1));
		if (ret < 0) {
			LOG_ERR("%s ret:%d", buf, ret);
			connect_ret = ret;
			goto maybe_retry;
		}

		/* set command handlers */
		ret = modem_cmd_handler_update_cmds(&mdata.cmd_handler_data,
						    cmd, ARRAY_SIZE(cmd), true);
		if (ret < 0) {
			connect_ret = ret;
			goto maybe_retry;
		}

		/* Wait for QI+OPEN */
		ret = k_sem_take(&mdata.sem_sock_conn, MDM_CMD_CONN_TIMEOUT);
		if (ret < 0) {
			LOG_ERR("Timeout waiting for socket open");
			connect_ret = ret;
			goto maybe_retry;
		}

		ret = modem_cmd_handler_get_error(&mdata.cmd_handler_data);
		if (ret != 0) {
			connect_ret = ret;
			goto maybe_retry;
		}

		/* Connected successfully. */
		memcpy(&sock->dst, addr, addrlen);
		sock->is_connected = true;
		errno = 0;
		return 0;

maybe_retry:
		(void) modem_cmd_handler_update_cmds(&mdata.cmd_handler_data,
						     NULL, 0U, false);
		(void)socket_close_cmd(sock);

		if (attempt + 1 < MDM_SOCKET_OPEN_ATTEMPTS) {
			LOG_WRN("Socket connect attempt %d failed ret=%d, retrying after recovery",
				attempt + 1, connect_ret);
			continue;
		}

		LOG_ERR("Closing the socket!!!");
		socket_close(sock);
		errno = -connect_ret;
		return -1;
	}

	errno = -connect_ret;
	return -1;
}

/* Func: offload_close
 * Desc: This function closes the connection with the remote client and
 * frees the socket.
 */
static int offload_close(void *obj)
{
	struct modem_socket *sock = (struct modem_socket *) obj;

	/* Make sure socket is allocated */
	if (modem_socket_is_allocated(&mdata.socket_config, sock) == false) {
		return 0;
	}

	/* Close the socket only if it is connected. */
	if (sock->is_connected) {
		socket_close(sock);
	}

	return 0;
}

/* Func: offload_sendmsg
 * Desc: This function sends messages to the modem.
 */
static ssize_t offload_sendmsg(void *obj, const struct msghdr *msg, int flags)
{
	ssize_t sent = 0;
	int rc;

	LOG_DBG("msg_iovlen:%zd flags:%d", msg->msg_iovlen, flags);

	for (int i = 0; i < msg->msg_iovlen; i++) {
		const char *buf = msg->msg_iov[i].iov_base;
		size_t len	= msg->msg_iov[i].iov_len;

		while (len > 0) {
			rc = offload_sendto(obj, buf, len, flags,
					    msg->msg_name, msg->msg_namelen);
			if (rc < 0) {
				if (rc == -EAGAIN) {
					k_sleep(MDM_SENDMSG_SLEEP);
				} else {
					sent = rc;
					break;
				}
			} else {
				sent += rc;
				buf += rc;
				len -= rc;
			}
		}
	}

	return (ssize_t) sent;
}

/* Func: modem_rx
 * Desc: Thread to process all messages received from the Modem.
 */
static void modem_rx(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	while (true) {

		/* Wait for incoming data */
		modem_iface_uart_rx_wait(&mctx.iface, K_FOREVER);

		modem_cmd_handler_process(&mctx.cmd_handler, &mctx.iface);
	}
}

/* Func: modem_rssi_query_work
 * Desc: Routine to get Modem RSSI.
 */
static void modem_rssi_query_work(struct k_work *work)
{
	struct modem_cmd cmd  = MODEM_CMD("+CSQ: ", on_cmd_atcmdinfo_rssi_csq, 2U, ",");
	static char *send_cmd = "AT+CSQ";
	int ret;

	/* query modem RSSI */
	ret = modem_cmd_send(&mctx.iface, &mctx.cmd_handler,
			     &cmd, 1U, send_cmd, &mdata.sem_response,
			     MDM_CMD_TIMEOUT);
	if (ret < 0) {
		LOG_ERR("AT+CSQ ret:%d", ret);
	}

	/* Do not keep probing RSSI in the background during the PSM harness.
	 * Those unsolicited AT+CSQ transactions collide with the idle window
	 * we are trying to validate.
	 */
	ARG_UNUSED(work);
}

/* Func: pin_init
 * Desc: Boot up the Modem.
 */
static void pin_init(void)
{
#if !DT_INST_NODE_HAS_PROP(0, mdm_reset_gpios)
	int ret = k_sem_take(&mdata.sem_pin_busy, K_SECONDS(3));

	if (ret < 0) {
		LOG_DBG("Timeout pin_init()");
	}
#endif /* !DT_INST_NODE_HAS_PROP(0, mdm_reset_gpios) */
	LOG_INF("Setting Modem Pins");
	modem_ap_ready_set(true, "pin-init");
	modem_log_runtime_lines_dbg("pin_init entry");

#if DT_INST_NODE_HAS_PROP(0, mdm_wdisable_gpios)
	LOG_INF("Deactivate W Disable");
	gpio_pin_set_dt(&wdisable_gpio, 0);
	k_sleep(K_MSEC(250));
#endif

	/* NOTE: Per the BG95 document, the Reset pin is internally connected to the
	 * Power key pin.
	 */

	/* MDM_POWER -> 1 for 500-1000 msec. */
	gpio_pin_set_dt(&power_gpio, 1);
	k_sleep(K_MSEC(750));

	/* MDM_POWER -> 0 and wait for ~2secs as UART remains in "inactive" state
	 * for some time after the power signal is enabled.
	 */
	gpio_pin_set_dt(&power_gpio, 0);
	k_sleep(K_SECONDS(2));

	modem_log_runtime_lines_dbg("pin_init exit");
	LOG_INF("... Done!");

#if !DT_INST_NODE_HAS_PROP(0, mdm_reset_gpios)
	k_sem_give(&mdata.sem_pin_busy);
#endif /* !DT_INST_NODE_HAS_PROP(0, mdm_reset_gpios) */
}

MODEM_CMD_DEFINE(on_cmd_unsol_normal_power_down)
{
	mdata.setup_complete = false;
	mdata.runtime_wake_pending = false;
	LOG_INF("Modem powering off. Re-power modem...");
	pin_init();

	return 0;
}

static const struct modem_cmd response_cmds[] = {
	MODEM_CMD("OK", on_cmd_ok, 0U, ""),
	MODEM_CMD("ERROR", on_cmd_error, 0U, ""),
	MODEM_CMD("+CME ERROR: ", on_cmd_exterror, 1U, ""),
};

static const struct modem_cmd unsol_cmds[] = {
	MODEM_CMD("+QIURC: \"recv\",",	   on_cmd_unsol_recv,  1U, ""),
	MODEM_CMD("+QIURC: \"closed\",",   on_cmd_unsol_close, 1U, ""),
#if defined(CONFIG_DNS_RESOLVER)
	MODEM_CMD_ARGS_MAX("+QIURC: \"dnsgip\",", on_cmd_unsol_dnsgip, 1U, 3U, ","),
#endif
	MODEM_CMD(MDM_UNSOL_RDY, on_cmd_unsol_rdy, 0U, ""),
	MODEM_CMD("NORMAL POWER DOWN", on_cmd_unsol_normal_power_down, 0U, ""),
};

#if defined(CONFIG_DNS_RESOLVER)
static int offload_getaddrinfo(const char *node, const char *service,
			       const struct zsock_addrinfo *hints,
			       struct zsock_addrinfo **res)
{
	char sendbuf[sizeof("AT+QIDNSGIP=1,\"\"") + 128];
	uint32_t port = 0U;
	int ret;
	int attempt;

	ret = modem_ensure_ready();
	if (ret < 0) {
		return DNS_EAI_FAIL;
	}

	memset(&dns_result, 0, sizeof(dns_result));
	memset(&dns_result_addr, 0, sizeof(dns_result_addr));
	memset(dns_result_canonname, 0, sizeof(dns_result_canonname));

	dns_result.ai_family = AF_INET;
	dns_result.ai_addr = (struct sockaddr *)&dns_result_addr;
	dns_result.ai_addrlen = sizeof(dns_result_addr);
	dns_result.ai_canonname = dns_result_canonname;
	dns_result_addr.sin_family = AF_INET;

	if (service != NULL) {
		port = ATOI(service, 0U, "service");
		if (port < 1U || port > UINT16_MAX) {
			return DNS_EAI_SERVICE;
		}

		dns_result_addr.sin_port = htons(port);
	}

	if (net_addr_pton(AF_INET, node, &dns_result_addr.sin_addr) == 0) {
		strncpy(dns_result_canonname, node, sizeof(dns_result_canonname) - 1);
		*res = &dns_result;
		return 0;
	}

	if (hints != NULL && (hints->ai_flags & AI_NUMERICHOST) != 0) {
		return DNS_EAI_NONAME;
	}

	snprintk(sendbuf, sizeof(sendbuf), "AT+QIDNSGIP=1,\"%s\"", node);
	for (attempt = 0; attempt < 2; attempt++) {
		ret = modem_ensure_ready();
		if (ret < 0) {
			LOG_WRN("DNS preflight modem_ready failed: %d", ret);
			return DNS_EAI_FAIL;
		}

		k_sem_reset(&mdata.sem_dns);
		mdata.dns_result = DNS_EAI_AGAIN;
		mdata.dns_modem_result = 0;
		mdata.dns_query_active = true;

		ret = modem_cmd_send(&mctx.iface, &mctx.cmd_handler,
				     NULL, 0U, sendbuf, &mdata.sem_response,
				     MDM_CMD_TIMEOUT);
		if (ret < 0) {
			mdata.dns_query_active = false;
			if (attempt == 0 && !mdata.gps_active) {
				LOG_WRN("DNS command send failed: %d; preparing runtime wake",
					ret);
				ret = modem_runtime_prepare();
				if (ret == 0) {
					continue;
				}
				LOG_WRN("Runtime wake prepare before DNS resend failed: %d", ret);
			} else if (mdata.gps_active) {
				LOG_DBG("DNS send failed during GPS session, skipping wake");
			}
			return DNS_EAI_AGAIN;
		}

		ret = k_sem_take(&mdata.sem_dns, K_SECONDS(65));
		if (ret < 0) {
			mdata.dns_query_active = false;
			if (attempt == 0 && !mdata.gps_active) {
				LOG_WRN("DNS response timeout; preparing runtime wake");
				ret = modem_runtime_prepare();
				if (ret == 0) {
					continue;
				}
				LOG_WRN("Runtime wake prepare after DNS timeout failed: %d", ret);
			} else if (mdata.gps_active) {
				LOG_DBG("DNS timeout during GPS session, skipping wake");
			}
			return DNS_EAI_AGAIN;
		}

		if (mdata.dns_result == 0) {
			*res = &dns_result;
			return 0;
		}

		if (attempt == 0 &&
		    (mdata.dns_modem_result == 557 ||
		     mdata.dns_modem_result == 558 ||
		     mdata.dns_modem_result == 561)) {
			LOG_WRN("Retrying DNS after modem runtime wake prepare, result=%d",
				mdata.dns_modem_result);
			ret = modem_runtime_prepare();
			if (ret == 0) {
				continue;
			}
			LOG_WRN("Runtime wake prepare before DNS retry failed: %d", ret);
		}

		return mdata.dns_result;
	}

	return DNS_EAI_AGAIN;
}

static void offload_freeaddrinfo(struct zsock_addrinfo *res)
{
	ARG_UNUSED(res);
}

static const struct socket_dns_offload offload_dns_ops = {
	.getaddrinfo = offload_getaddrinfo,
	.freeaddrinfo = offload_freeaddrinfo,
};
#endif

/* Commands sent to the modem to set it up at boot time. */
static const struct setup_cmd setup_cmds[] = {
	SETUP_CMD_NOHANDLE("ATE0"),
	SETUP_CMD_NOHANDLE("ATH"),
	SETUP_CMD_NOHANDLE("AT+CMEE=1"),

	/* Commands to read info from the modem (things like IMEI, Model etc). */
	SETUP_CMD("AT+CGMI", "", on_cmd_atcmdinfo_manufacturer, 0U, ""),
	SETUP_CMD("AT+CGMM", "", on_cmd_atcmdinfo_model, 0U, ""),
	SETUP_CMD("AT+CGMR", "", on_cmd_atcmdinfo_revision, 0U, ""),
	SETUP_CMD("AT+CGSN", "", on_cmd_atcmdinfo_imei, 0U, ""),
#if defined(CONFIG_MODEM_SIM_NUMBERS)
	SETUP_CMD("AT+CIMI", "", on_cmd_atcmdinfo_imsi, 0U, ""),
	SETUP_CMD("AT+QCCID", "", on_cmd_atcmdinfo_iccid, 0U, ""),
#endif /* #if defined(CONFIG_MODEM_SIM_NUMBERS) */
	SETUP_CMD_NOHANDLE("AT+QICSGP=1,1,\"" MDM_APN "\",\""
			   MDM_USERNAME "\",\"" MDM_PASSWORD "\",1"),
	/* Re-enable short PSM timers on the now-working UDP offload baseline. */
	SETUP_CMD_NOHANDLE("AT+QCFG=\"psm/urc\",1"),
	SETUP_CMD_NOHANDLE("AT+QCFG=\"apready\",1,1,500"),
	SETUP_CMD_NOHANDLE("AT+QSCLK=0"),
};

/* Func: modem_pdp_context_active
 * Desc: This helper function is called from modem_setup, and is
 * used to open the PDP context. If there is trouble activating the
 * PDP context, we try to deactivate and reactivate MDM_PDP_ACT_RETRY_COUNT times.
 * If it fails, we return an error.
 */
static int modem_pdp_context_activate(void)
{
	int ret;
	int retry_count = 0;

	do {
		ret = modem_cmd_send(&mctx.iface, &mctx.cmd_handler,
				     NULL, 0U, "AT+QIACT=1", &mdata.sem_response,
				     MDM_CMD_TIMEOUT);
		if (ret == 0) {
			ret = modem_pdp_context_verify();
			if (ret == 0) {
				return 0;
			}
		}

		if (ret != -EIO && ret != -ENETDOWN) {
			return ret;
		}

		if (retry_count >= MDM_PDP_ACT_RETRY_COUNT) {
			break;
		}

		ret = modem_cmd_send(&mctx.iface, &mctx.cmd_handler,
				     NULL, 0U, "AT+QIDEACT=1", &mdata.sem_response,
				     MDM_CMD_TIMEOUT);
		if (ret != 0) {
			return ret;
		}

		retry_count++;
		LOG_WRN("Retrying PDP activation after verification failure (%d/%d)",
			retry_count, MDM_PDP_ACT_RETRY_COUNT);
	} while (retry_count <= MDM_PDP_ACT_RETRY_COUNT);

	LOG_ERR("Retried activating/deactivating too many times.");
	return ret;
}

static int modem_runtime_at_ping(k_timeout_t timeout)
{
	return modem_cmd_send(&mctx.iface, &mctx.cmd_handler,
			      NULL, 0U, "AT", &mdata.sem_response,
			      timeout);
}

#if DT_INST_NODE_HAS_PROP(0, mdm_dtr_gpios)
static void modem_runtime_dtr_nudge(void)
{
	LOG_INF("Pulsing DTR for runtime wake");
	gpio_pin_set_dt(&dtr_gpio, 1);
	k_sleep(K_MSEC(100));
	gpio_pin_set_dt(&dtr_gpio, 0);
	k_sleep(K_MSEC(100));
}
#endif

static void modem_runtime_power_wake(void)
{
	modem_log_runtime_lines_dbg("before power-key wake");
	LOG_INF("Pulsing modem power key for runtime wake");
	gpio_pin_set_dt(&power_gpio, 1);
	k_sleep(K_MSEC(750));
	gpio_pin_set_dt(&power_gpio, 0);
	k_sleep(K_MSEC(60));
	if (modem_wait_for_runtime_lines(K_SECONDS(5)) == 0) {
		modem_log_runtime_lines_dbg("after power-key wake");
	} else {
		k_sleep(K_SECONDS(2));
	}
	/* Stock firmware uses a noticeably longer settle window around wake.
	 * Going straight from STATUS-high to AT probing is still provoking
	 * clunky resume behavior on this board.
	 */
	k_sleep(MDM_RUNTIME_POST_PWRKEY_SETTLE);
}

static void modem_ap_ready_set(bool host_awake, const char *reason)
{
#if DT_INST_NODE_HAS_PROP(0, mdm_ap_ready_gpios)
	gpio_pin_set_dt(&ap_ready_gpio, host_awake ? 1 : 0);
	LOG_INF("AP_READY host_%s%s%s",
		host_awake ? "awake" : "sleep",
		reason != NULL ? " reason=" : "",
		reason != NULL ? reason : "");
#else
	ARG_UNUSED(host_awake);
	ARG_UNUSED(reason);
#endif
}

static int modem_runtime_restore(void)
{
	int ret;

	/* Persistent modem setup should already be in place. On runtime wake, the
	 * only thing we need back is a live PDP context for socket reopen.
	 */

	ret = modem_pdp_context_activate();
	if (ret < 0) {
		LOG_ERR("Runtime PDP activation failed: %d", ret);
		return ret;
	}

	mdata.setup_complete = true;
	mdata.runtime_wake_pending = false;
	runtime_native_wake_count++;

	if (psm_policy_dirty) {
		ret = bg9x_apply_psm_preference("Runtime restore");
		if (ret < 0) {
			LOG_WRN("Runtime restore: PSM apply failed: %d", ret);
		}
	} else {
		LOG_INF("Runtime restore: PSM policy already current");
	}

	LOG_INF("Runtime modem restore complete (native=%u emergency=%u)",
		runtime_native_wake_count,
		runtime_emergency_reinit_count);
	return 0;
}

static int modem_runtime_prepare(void)
{
	int ret;
	bool try_dtr = true;
	bool direct_probe_allowed = true;
	k_timeout_t direct_probe_timeout = K_SECONDS(2);
	bool status_high = true;
	bool psm_asserted = false;

	LOG_INF("Preparing modem for post-PSM traffic");
	modem_ap_ready_set(true, "runtime-wake");
	modem_log_runtime_lines_info("runtime wake entry");

#if DT_INST_NODE_HAS_PROP(0, mdm_status_gpios)
	status_high = modem_runtime_line_asserted(&status_gpio);
	if (!status_high) {
		LOG_INF("STATUS low at runtime wake entry; skipping speculative AT probe");
		direct_probe_allowed = false;
	}
#endif

#if DT_INST_NODE_HAS_PROP(0, mdm_psm_gpios)
	psm_asserted = modem_runtime_line_asserted(&psm_gpio);
	if (psm_asserted && !status_high) {
		LOG_INF("PSM line asserted at runtime wake entry; forcing explicit wake path");
		direct_probe_allowed = false;
	}
#endif

	if (mdata.runtime_wake_pending) {
		LOG_INF("Runtime wake pending; trying direct AT probe before wake escalation");
		if (direct_probe_allowed) {
			k_sleep(K_MSEC(250));
			direct_probe_timeout = K_SECONDS(3);
		}
	}

	if (direct_probe_allowed) {
		if (psm_asserted) {
			LOG_INF("PSM line asserted with STATUS high; trying direct AT probe before wake pulse");
			k_sleep(K_MSEC(250));
		}
		ret = modem_runtime_at_ping(direct_probe_timeout);
		if (ret == 0) {
			LOG_INF("Runtime AT probe succeeded without wake pulse");
			ret = modem_runtime_restore();
			if (ret == 0) {
				k_sleep(MDM_RUNTIME_RECOVER_SETTLE);
				return 0;
			}
		}
		LOG_WRN("Runtime AT probe failed: %d", ret);
	}

	if (mdata.runtime_wake_pending) {
		LOG_INF("Runtime wake pending; skipping DTR wake trial");
		try_dtr = false;
	}

#if DT_INST_NODE_HAS_PROP(0, mdm_dtr_gpios)
	if (!status_high) {
		LOG_INF("STATUS low at runtime wake; skipping DTR wake");
		try_dtr = false;
	}
	if (try_dtr) {
	modem_runtime_dtr_nudge();
		modem_log_runtime_lines_dbg("after DTR wake");
	if (modem_wait_for_runtime_lines(K_SECONDS(2)) < 0) {
		LOG_WRN("Awake-line wait after DTR wake timed out");
	}
	ret = modem_runtime_at_ping(K_SECONDS(2));
	if (ret == 0) {
		LOG_INF("AT responded after DTR wake");
		ret = modem_runtime_restore();
		if (ret == 0) {
			k_sleep(MDM_RUNTIME_RECOVER_SETTLE);
			return 0;
		}
	}
	LOG_WRN("DTR wake path did not restore modem state: %d", ret);
	}
	#endif

	modem_runtime_power_wake();
	ret = modem_runtime_at_ping(K_SECONDS(3));
	if (ret == 0) {
		LOG_INF("AT responded after power-key wake");
		ret = modem_runtime_restore();
		if (ret == 0) {
			k_sleep(MDM_RUNTIME_RECOVER_SETTLE);
			return 0;
		}
	}
	LOG_WRN("Power-key wake path did not restore modem state: %d", ret);
	return modem_runtime_emergency_reinit();
}

static int modem_runtime_emergency_reinit(void)
{
	int ret;

	runtime_emergency_reinit_count++;
	ret = modem_setup();
	if (ret < 0) {
		LOG_ERR("Full modem setup fallback failed: %d", ret);
		return ret;
	}

	LOG_WRN("Runtime wake prepare fell back to full modem setup (native=%u emergency=%u)",
		runtime_native_wake_count,
		runtime_emergency_reinit_count);
	k_sleep(MDM_RUNTIME_RECOVER_SETTLE);
	return 0;
}

/* Func: modem_setup
 * Desc: This function is used to setup the modem from zero. The idea
 * is that this function will be called right after the modem is
 * powered on to do the stuff necessary to talk to the modem.
 */
static int modem_setup(void)
{
	int ret = 0, counter;
	int rssi_retry_count = 0, init_retry_count = 0;

	mdata.setup_complete = false;
	mdata.runtime_wake_pending = false;

	/* Setup the pins to ensure that Modem is enabled. */
	pin_init();

restart:

	counter = 0;

	/* stop RSSI delay work */
	k_work_cancel_delayable(&mdata.rssi_query_work);

	/* Let the modem respond. */
	LOG_INF("Waiting for modem to respond");
	ret = k_sem_take(&mdata.sem_response, MDM_MAX_BOOT_TIME);
	if (ret < 0) {
		LOG_ERR("Timeout waiting for RDY");
		goto error;
	}

	/* Run setup commands on the modem. */
	ret = modem_cmd_handler_setup_cmds(&mctx.iface, &mctx.cmd_handler,
					   setup_cmds, ARRAY_SIZE(setup_cmds),
					   &mdata.sem_response, MDM_REGISTRATION_TIMEOUT);
	if (ret < 0) {
		goto error;
	}

restart_rssi:

	/* query modem RSSI */
	modem_rssi_query_work(NULL);
	k_sleep(MDM_WAIT_FOR_RSSI_DELAY);

	/* Keep trying to read RSSI until we get a valid value - Eventually, exit. */
	while (counter++ < MDM_WAIT_FOR_RSSI_COUNT &&
	      (mdata.mdm_rssi >= 0 || mdata.mdm_rssi <= -1000)) {
		modem_rssi_query_work(NULL);
		k_sleep(MDM_WAIT_FOR_RSSI_DELAY);
	}

	/* Is the RSSI invalid ? */
	if (mdata.mdm_rssi >= 0 || mdata.mdm_rssi <= -1000) {
		rssi_retry_count++;

		if (rssi_retry_count >= MDM_NETWORK_RETRY_COUNT) {
			LOG_ERR("Failed network init. Too many attempts!");
			ret = -ENETUNREACH;
			goto error;
		}

		/* Try again! */
		LOG_ERR("Failed network init. Restarting process.");
		counter = 0;
		goto restart_rssi;
	}

	/* Network is ready. Keep the background RSSI work disabled for the
	 * harness so the modem can stay idle without AT+CSQ traffic.
	 */
	LOG_INF("Network is ready.");

	/* Once the network is ready, we try to activate the PDP context. */
	ret = modem_pdp_context_activate();
	if (ret < 0 && init_retry_count++ < MDM_INIT_RETRY_COUNT) {
		LOG_ERR("Error activating modem with pdp context");
		goto restart;
	}

	mdata.setup_complete = true;
	mdata.runtime_wake_pending = false;

	ret = bg9x_apply_psm_preference("Initial setup");
	if (ret < 0) {
		LOG_WRN("Initial setup: PSM apply failed: %d", ret);
	}

error:
	return ret;
}

static const struct socket_op_vtable offload_socket_fd_op_vtable = {
	.fd_vtable = {
		.read	= offload_read,
		.write	= offload_write,
		.close	= offload_close,
		.ioctl	= offload_ioctl,
	},
	.bind		= NULL,
	.connect	= offload_connect,
	.sendto		= offload_sendto,
	.recvfrom	= offload_recvfrom,
	.listen		= NULL,
	.accept		= NULL,
	.sendmsg	= offload_sendmsg,
	.getsockopt	= offload_getsockopt,
	.setsockopt	= offload_setsockopt,
};

static int offload_socket(int family, int type, int proto);

/* Setup the Modem NET Interface. */
static void modem_net_iface_init(struct net_if *iface)
{
	const struct device *dev = net_if_get_device(iface);
	struct modem_data *data	 = dev->data;

	/* Direct socket offload used instead of net offload: */
	net_if_set_link_addr(iface, modem_get_mac(dev),
			     sizeof(data->mac_addr),
			     NET_LINK_ETHERNET);
	data->net_iface = iface;
#if defined(CONFIG_DNS_RESOLVER)
	socket_offload_dns_register(&offload_dns_ops);
#endif

	net_if_socket_offload_set(iface, offload_socket);
}

static struct offloaded_if_api api_funcs = {
	.iface_api.init = modem_net_iface_init,
};

static bool offload_is_supported(int family, int type, int proto)
{
	if (family != AF_INET &&
	    family != AF_INET6) {
		return false;
	}

	if (type == SOCK_STREAM && proto == IPPROTO_TCP) {
		return true;
	}

	if (type == SOCK_DGRAM && proto == IPPROTO_UDP) {
		return true;
	}

	return false;
}

static int offload_socket(int family, int type, int proto)
{
	int ret;
	struct modem_socket *sock;

	/* defer modem's socket create call to bind() */
	ret = modem_socket_get(&mdata.socket_config, family, type, proto);
	if (ret < 0) {
		errno = -ret;
		return -1;
	}

	sock = modem_socket_from_fd(&mdata.socket_config, ret);
	if (sock != NULL) {
		(void)offload_socket_set_flags(sock, 0);
	}

	errno = 0;
	return ret;
}

static int modem_init(const struct device *dev)
{
	int ret; ARG_UNUSED(dev);

#if !DT_INST_NODE_HAS_PROP(0, mdm_reset_gpios)
	k_sem_init(&mdata.sem_pin_busy,	 1, 1);
#endif /* !DT_INST_NODE_HAS_PROP(0, mdm_reset_gpios) */
	k_sem_init(&mdata.sem_response,	 0, 1);
	k_sem_init(&mdata.sem_tx_ready,	 0, 1);
	k_sem_init(&mdata.sem_sock_conn, 0, 1);
	k_sem_init(&mdata.sem_dns, 0, 1);
	k_work_queue_start(&modem_workq, modem_workq_stack,
			   K_KERNEL_STACK_SIZEOF(modem_workq_stack),
			   K_PRIO_COOP(7), NULL);

	/* socket config */
	ret = modem_socket_init(&mdata.socket_config, &mdata.sockets[0], ARRAY_SIZE(mdata.sockets),
				MDM_BASE_SOCKET_NUM, true, &offload_socket_fd_op_vtable);
	if (ret < 0) {
		goto error;
	}

	/* cmd handler setup */
	const struct modem_cmd_handler_config cmd_handler_config = {
		.match_buf = &mdata.cmd_match_buf[0],
		.match_buf_len = sizeof(mdata.cmd_match_buf),
		.buf_pool = &mdm_recv_pool,
		.alloc_timeout = BUF_ALLOC_TIMEOUT,
		.eol = "\r\n",
		.user_data = NULL,
		.response_cmds = response_cmds,
		.response_cmds_len = ARRAY_SIZE(response_cmds),
		.unsol_cmds = unsol_cmds,
		.unsol_cmds_len = ARRAY_SIZE(unsol_cmds),
	};

	ret = modem_cmd_handler_init(&mctx.cmd_handler, &mdata.cmd_handler_data,
				     &cmd_handler_config);
	if (ret < 0) {
		goto error;
	}

	/* modem interface */
	const struct modem_iface_uart_config uart_config = {
		.rx_rb_buf = &mdata.iface_rb_buf[0],
		.rx_rb_buf_len = sizeof(mdata.iface_rb_buf),
		.dev = MDM_UART_DEV,
		.hw_flow_control = DT_PROP(MDM_UART_NODE, hw_flow_control),
	};

	ret = modem_iface_uart_init(&mctx.iface, &mdata.iface_data, &uart_config);
	if (ret < 0) {
		goto error;
	}

	/* modem data storage */
	mctx.data_manufacturer = mdata.mdm_manufacturer;
	mctx.data_model	       = mdata.mdm_model;
	mctx.data_revision     = mdata.mdm_revision;
	mctx.data_imei	       = mdata.mdm_imei;
#if defined(CONFIG_MODEM_SIM_NUMBERS)
	mctx.data_imsi	       = mdata.mdm_imsi;
	mctx.data_iccid	       = mdata.mdm_iccid;
#endif /* #if defined(CONFIG_MODEM_SIM_NUMBERS) */
	mctx.data_rssi = &mdata.mdm_rssi;

	/* pin setup */
	ret = gpio_pin_configure_dt(&power_gpio, GPIO_OUTPUT_LOW);
	if (ret < 0) {
		LOG_ERR("Failed to configure %s pin", "power");
		goto error;
	}

#if DT_INST_NODE_HAS_PROP(0, mdm_reset_gpios)
	ret = gpio_pin_configure_dt(&reset_gpio, GPIO_OUTPUT_LOW);
	if (ret < 0) {
		LOG_ERR("Failed to configure %s pin", "reset");
		goto error;
	}
#endif

#if DT_INST_NODE_HAS_PROP(0, mdm_dtr_gpios)
	ret = gpio_pin_configure_dt(&dtr_gpio, GPIO_OUTPUT_LOW);
	if (ret < 0) {
		LOG_ERR("Failed to configure %s pin", "dtr");
		goto error;
	}
#endif

#if DT_INST_NODE_HAS_PROP(0, mdm_status_gpios)
	ret = gpio_pin_configure_dt(&status_gpio, GPIO_INPUT);
	if (ret < 0) {
		LOG_ERR("Failed to configure %s pin", "status");
		goto error;
	}
#endif

#if DT_INST_NODE_HAS_PROP(0, mdm_ap_ready_gpios)
	ret = gpio_pin_configure_dt(&ap_ready_gpio, GPIO_OUTPUT_HIGH);
	if (ret < 0) {
		LOG_ERR("Failed to configure %s pin", "ap_ready");
		goto error;
	}
#endif

#if DT_INST_NODE_HAS_PROP(0, mdm_psm_gpios)
	ret = gpio_pin_configure_dt(&psm_gpio, GPIO_INPUT);
	if (ret < 0) {
		LOG_ERR("Failed to configure %s pin", "psm");
		goto error;
	}
#endif

#if DT_INST_NODE_HAS_PROP(0, mdm_ri_gpios)
	ret = gpio_pin_configure_dt(&ri_gpio, GPIO_INPUT);
	if (ret < 0) {
		LOG_ERR("Failed to configure %s pin", "ri");
		goto error;
	}
#endif

#if DT_INST_NODE_HAS_PROP(0, mdm_wdisable_gpios)
	ret = gpio_pin_configure_dt(&wdisable_gpio, GPIO_OUTPUT_LOW);
	if (ret < 0) {
		LOG_ERR("Failed to configure %s pin", "wdisable");
		goto error;
	}
#endif

	/* modem context setup */
	mctx.driver_data       = &mdata;

	ret = modem_context_register(&mctx);
	if (ret < 0) {
		LOG_ERR("Error registering modem context: %d", ret);
		goto error;
	}

	/* start RX thread */
	k_thread_create(&modem_rx_thread, modem_rx_stack,
			K_KERNEL_STACK_SIZEOF(modem_rx_stack),
			modem_rx,
			NULL, NULL, NULL, K_PRIO_COOP(7), 0, K_NO_WAIT);

	/* Init RSSI query */
	k_work_init_delayable(&mdata.rssi_query_work, modem_rssi_query_work);
	ret = modem_setup();
	if (ret < 0) {
		LOG_WRN("Initial modem setup failed: %d; deferring setup/wake to runtime", ret);
		return 0;
	}

	return 0;

error:
	return ret;
}

/* Register the device with the Networking stack. */
NET_DEVICE_DT_INST_OFFLOAD_DEFINE(0, modem_init, NULL,
				  &mdata, NULL,
				  CONFIG_MODEM_QUECTEL_BG9X_INIT_PRIORITY,
				  &api_funcs, MDM_MAX_DATA_LENGTH);

/* Register NET sockets. */
NET_SOCKET_OFFLOAD_REGISTER(quectel_bg9x, CONFIG_NET_SOCKETS_OFFLOAD_PRIORITY,
			    AF_UNSPEC, offload_is_supported, offload_socket);
