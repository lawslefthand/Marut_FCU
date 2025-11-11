/*
 * gps_parser.c
 *
 *  Created on: Oct 29, 2025
 *      Author: danba
 */


#include "stm32f4xx.h"
#include "gps_parser.h"
#include "string.h"


uint8_t rxBuffer[128] = { 0 };
uint8_t rxIndex = 0;
uint8_t rxData;
float nmeaLong;
float nmeaLat;
float utcTime;
char posStatus;
char northsouth;
char eastwest;
float decimalLong;
float decimalLat;
float gpsSpeed;
float course;
int numSats = 0;
float mslAlt = 0.0f;
int gpsQuality = 0;
int has_fix = 0;
int fix_type = 0;
float hdop = 0.0f;
uint32_t last_led_toggle = 0;
char unit;
uint32_t gps_send_counter = 0;

float nmeaToDecimal(float coordinate) {
	int degree = (int) (coordinate / 100);
	float minutes = coordinate - degree * 100;
	float decimalDegree = minutes / 60;
	float decimal = degree + decimalDegree;
	return decimal;
}

int gpsValidate(char *nmea) {
	char check[3];
	char calculatedString[3];
	int index;
	int calculatedCheck;

	index = 0;
	calculatedCheck = 0;

	if (nmea[index] == '$')
		index++;
	else
		return 0;

	while ((nmea[index] != 0) && (nmea[index] != '*') && (index < 75)) {
		calculatedCheck ^= nmea[index];
		index++;
	}

	if (index >= 75) {
		return 0;
	}

	if (nmea[index] == '*') {
		check[0] = nmea[index + 1];
		check[1] = nmea[index + 2];
		check[2] = 0;
	} else
		return 0;

	sprintf(calculatedString, "%02X", calculatedCheck);
	return ((calculatedString[0] == check[0])
			&& (calculatedString[1] == check[1])) ? 1 : 0;
}

void gpsParse(char *strParse) {
	int new_fix = 0;
	if (!strncmp(strParse, "$GPGGA", 6)) {

		sscanf(strParse, "$GPGGA,%f,%f,%c,%f,%c,%d,%d,%f,%f,%c", &utcTime,
				&nmeaLat, &northsouth, &nmeaLong, &eastwest, &gpsQuality,
				&numSats, &hdop, &mslAlt, &unit);
		decimalLat = nmeaToDecimal(nmeaLat);
		decimalLong = nmeaToDecimal(nmeaLong);

		if (gpsQuality > 0) {
			switch (gpsQuality) {
			case 0:
				fix_type = 0;
				break;
			case 1:
				fix_type = 3;
				break;
			case 2:
				fix_type = 4;
				break;
			default:
				fix_type = 3;
				break;
			}
			has_fix = 1;
			new_fix = 1;
			printf("GPS Fix Acquired from GGA! Quality: %d, Satellites: %d\n",
					gpsQuality, numSats);
		} else {
			fix_type = 0;
			has_fix = 0;
		}
	} else if (!strncmp(strParse, "$GPGLL", 6)) {
		sscanf(strParse, "$GPGLL,%f,%c,%f,%c,%f", &nmeaLat, &northsouth,
				&nmeaLong, &eastwest, &utcTime);
		decimalLat = nmeaToDecimal(nmeaLat);
		decimalLong = nmeaToDecimal(nmeaLong);
	} else if (!strncmp(strParse, "$GPRMC", 6)) {

		sscanf(strParse, "$GPRMC,%f,%c,%f,%c,%f,%c,%f,%f,%*s", &utcTime,
				&posStatus, &nmeaLat, &northsouth, &nmeaLong, &eastwest,
				&gpsSpeed, &course);
		decimalLat = nmeaToDecimal(nmeaLat);
		decimalLong = nmeaToDecimal(nmeaLong);
		new_fix = (posStatus == 'A') ? 1 : 0;
		if (new_fix != has_fix) {
			has_fix = new_fix;
			fix_type = new_fix ? 3 : 0;
			if (has_fix) {
				printf("GPS Fix Active! (Status: A)\n");
			} else {
				printf("No GPS Fix (Status: V) - Searching...\n");
			}
		}
	}

	if (has_fix && strncmp(strParse, "$GPRMC", 6) == 0) {

		float signedLat = (northsouth == 'S') ? -decimalLat : decimalLat;
		float signedLon = (eastwest == 'W') ? -decimalLong : decimalLong;
		printf(
				"Fix Active - Lat: %.6f, Lon: %.6f, Speed: %.2f knots, Course: %.1f°, Sats: %d, Alt: %.2f m\n",
				signedLat, signedLon, gpsSpeed, course, numSats, mslAlt);
	}
}
