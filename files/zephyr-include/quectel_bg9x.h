/*
 * Copyright (c) 2025 Willow Technologies
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_MODEM_QUECTEL_BG9X_H_
#define ZEPHYR_INCLUDE_DRIVERS_MODEM_QUECTEL_BG9X_H_

#include <stdbool.h>
#include <stdint.h>

struct bg9x_runtime_stats {
	uint32_t native_wake_count;
	uint32_t emergency_reinit_count;
};

struct bg9x_data_usage {
	bool valid;
	uint64_t tx_bytes;
	uint64_t rx_bytes;
};

struct bg9x_runtime_snapshot {
	bool at_ready;
	bool setup_complete;
	bool runtime_wake_pending;
	bool dns_query_active;
	bool pdp_active;
	bool gps_active;
	uint8_t allocated_sockets;
	uint8_t connected_sockets;
	int mdm_rssi;
	int at_probe_ret;
	int status_gpio;
	int ap_ready_gpio;
	int psm_gpio;
	int ri_gpio;
};

struct bg9x_gps_fix {
	/* fix validity */
	bool valid;
	uint8_t fix_type;           /* 0=none, 2=2D, 3=3D */

	/* position */
	double latitude;            /* decimal degrees, negative = south */
	double longitude;           /* decimal degrees, negative = west */
	float altitude;             /* meters above MSL */

	/* accuracy / dilution of precision */
	float hdop;                 /* horizontal DOP */
	float vdop;                 /* vertical DOP (from GSA) */
	float pdop;                 /* position DOP (from GSA) */

	/* motion */
	float speed_kmh;            /* km/h */
	float speed_knots;          /* knots */
	float course;               /* true course over ground, degrees 0-360 */
	float magnetic_course;      /* magnetic course, degrees (from VTG) */
	float magnetic_variation;   /* magnetic variation, degrees, + = east (from RMC) */

	/* satellites */
	uint8_t sats_in_use;        /* satellites used in fix (from QGPSLOC) */
	uint8_t sats_in_view;       /* total satellites visible (from GSV) */

	/* per-satellite detail (from GSV, up to 16) */
	uint8_t sat_count;
	struct {
		uint8_t prn;
		uint8_t elevation;
		uint16_t azimuth;
		uint8_t snr;
	} sat_detail[16];

	/* UTC time from GPS */
	uint8_t utc_hour;
	uint8_t utc_min;
	uint8_t utc_sec;
	uint16_t utc_ms;
	uint8_t day;
	uint8_t month;
	uint16_t year;

	/* acquisition stats */
	uint32_t fix_time_ms;

	/* raw NMEA sentences (for debug/logging) */
	char nmea_gga[120];
	char nmea_rmc[120];
};

struct bg9x_cell_info {
	bool valid;
	uint16_t mcc;
	uint16_t mnc;
	uint32_t cell_id;
	uint16_t pci;
	uint16_t tac;
	uint32_t earfcn;
	uint8_t band;
	int16_t rsrp;
	int16_t rsrq;
	int16_t rssi;
	int16_t sinr;
	char rat[8];
};

struct bg9x_network_time {
	bool valid;
	uint16_t year;
	uint8_t month;
	uint8_t day;
	uint8_t hour;
	uint8_t min;
	uint8_t sec;
	int8_t tz_quarter_hours;    /* timezone offset in 15-min increments */
};

int bg9x_runtime_stats_get(struct bg9x_runtime_stats *stats);
int bg9x_data_usage_get(struct bg9x_data_usage *usage);
int bg9x_runtime_snapshot_get(struct bg9x_runtime_snapshot *snapshot, bool probe_at);
int bg9x_connected_socket_qistate_get(bool prepare_on_timeout, int *sock_id, bool *present,
				      char *resp, size_t resp_size);
int bg9x_connected_socket_reopen(void);
int bg9x_connected_socket_resume_prepare(bool *present, bool *reopened,
					 char *resp, size_t resp_size);
int bg9x_runtime_prepare(void);
int bg9x_utc_day_get(uint32_t *utc_day);
int bg9x_cycle_begin(void);
int bg9x_gps_xtra_policy_set(bool enabled, uint8_t retention_days, uint8_t mode);
int bg9x_gps_session_begin(void);
int bg9x_gps_session_end(void);
int bg9x_gps_get_fix(struct bg9x_gps_fix *fix, int timeout_s);
int bg9x_cell_info_get(struct bg9x_cell_info *info);
int bg9x_network_time_get(struct bg9x_network_time *time);
int bg9x_at_cmd_passthrough(const char *cmd, char *resp, size_t resp_size);
int bg9x_psm_set(bool enable);
int bg9x_psm_set_interval(bool enable, uint32_t wake_seconds);

#endif /* ZEPHYR_INCLUDE_DRIVERS_MODEM_QUECTEL_BG9X_H_ */
