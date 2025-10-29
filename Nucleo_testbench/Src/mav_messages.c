/*
 * mav_messages.c
 *
 *  Created on: Oct 29, 2025
 *      Author: danba
 */

#include "mav_messages.h"
#include "string.h"
#include "stm32f4xx.h"
#include "mavlink/common/mavlink.h"
#include "bmp280.h"
#include "mpu6050_port.h"
#include "qmcr5883l.h"

extern UART_HandleTypeDef huart2;

extern uint8_t rxBuffer[128];
extern uint8_t rxIndex;
extern uint8_t rxData;
extern float nmeaLong;
extern float nmeaLat;
extern float utcTime;
extern char posStatus;
extern char northsouth;
extern char eastwest;
extern float decimalLong;
extern float decimalLat;
extern float gpsSpeed;
extern float course;
extern int numSats;
extern float mslAlt;
extern int gpsQuality;
extern int has_fix;
extern int fix_type;
extern float hdop;
extern uint32_t last_led_toggle;
extern char unit;
extern uint32_t gps_send_counter;

void send_heartbeat(void) {
	mavlink_message_t msg;
	uint8_t buf[MAVLINK_MAX_PACKET_LEN];

	mavlink_msg_heartbeat_pack(1, 200, &msg, MAV_TYPE_GENERIC,
			MAV_AUTOPILOT_GENERIC, MAV_MODE_MANUAL_ARMED, 0, MAV_STATE_ACTIVE);

	uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);

	HAL_UART_Transmit(&huart2, buf, len, HAL_MAX_DELAY);
}

void send_attitude(void) {
	mavlink_message_t msg;
	uint8_t buf[MAVLINK_MAX_PACKET_LEN];
	uint16_t len;

	uint8_t my_system_id = 1;
	uint8_t my_component_id = 200;

	mavlink_msg_attitude_pack(my_system_id, my_component_id, &msg,
			HAL_GetTick(), mpu_accel_read(0) * (3.14 / 180),
			mpu_accel_read(1) * (3.14 / 180), qmc_mag_read() * (3.14 / 180),
			3.0f, 3.0f, 3.0f);
	len = mavlink_msg_to_send_buffer(buf, &msg);
	HAL_UART_Transmit(&huart2, buf, len, HAL_MAX_DELAY);
}

void send_gps_raw_int(void) {
	mavlink_message_t msg;
	uint8_t buf[MAVLINK_MAX_PACKET_LEN];
	uint16_t len;

	uint8_t my_system_id = 1;
	uint8_t my_component_id = 200;

	int32_t lat_int = (int32_t) (decimalLat * 1E7);
	int32_t lon_int = (int32_t) (decimalLong * 1E7);

	int32_t alt_mm = (int32_t) (mslAlt * 1000.0f);

	float speed_ms = gpsSpeed * 0.514444f;
	uint16_t vel_cm_s = (uint16_t) (speed_ms * 100.0f);
	uint16_t cog_cdeg = (uint16_t) (course * 100.0f);
	uint16_t eph = (hdop > 0.0f) ? (uint16_t) (hdop * 100.0f) : UINT16_MAX;
	uint16_t epv = UINT16_MAX;

	mavlink_msg_gps_raw_int_pack(my_system_id, my_component_id, &msg,
			(uint64_t) HAL_GetTick() * 1000ULL, fix_type, lat_int, lon_int,

			alt_mm,

			eph, epv, vel_cm_s, cog_cdeg, (uint8_t) numSats,

			alt_mm,

			UINT32_MAX,
			UINT32_MAX,
			UINT32_MAX,
			UINT32_MAX, 0);

	len = mavlink_msg_to_send_buffer(buf, &msg);
	HAL_UART_Transmit(&huart2, buf, len, HAL_MAX_DELAY);
	printf("Sent GPS MAVLink: Fix %d, Lat %.6f, Lon %.6f, Sats %d\n", fix_type,
			decimalLat, decimalLong, numSats);
}

void send_global_position_int(void) {
	mavlink_message_t msg;
	uint8_t buf[MAVLINK_MAX_PACKET_LEN];

	uint8_t my_system_id = 1;
	uint8_t my_component_id = 200;
	uint32_t time_boot_ms = HAL_GetTick();

	float relative_alt_meters = altitude_calc();

	int32_t lat = (int32_t) (decimalLat * 1E7);
	int32_t lon = (int32_t) (decimalLong * 1E7);

	int32_t alt = (int32_t) (mslAlt * 1000.0f);

	int32_t relative_alt_mm = (int32_t) (relative_alt_meters * 1000.0f);

	int16_t vx = 0;
	int16_t vy = 0;
	int16_t vz = 0;

	uint16_t hdg = (uint16_t) (course * 100.0f);

	mavlink_msg_global_position_int_pack(my_system_id, my_component_id, &msg,
			time_boot_ms, lat, lon, alt, relative_alt_mm, vx, vy, vz, hdg);

	uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
	HAL_UART_Transmit(&huart2, buf, len, HAL_MAX_DELAY);
}
