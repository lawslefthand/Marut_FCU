/*
 * gps_parser.h
 *
 *  Created on: Oct 29, 2025
 *      Author: danba
 */

#ifndef INC_GPS_PARSER_H_
#define INC_GPS_PARSER_H_

float nmeaToDecimal(float coordinate);
int gpsValidate(char *nmea);
void gpsParse(char *strParse);

#endif /* INC_GPS_PARSER_H_ */
