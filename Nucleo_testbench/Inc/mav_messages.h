/*
 * mav_messages.h
 *
 *  Created on: Oct 29, 2025
 *      Author: danba
 */

#ifndef INC_MAV_MESSAGES_H_
#define INC_MAV_MESSAGES_H_

void send_heartbeat(void);
void send_attitude(void);
void send_gps_raw_int(void);
void send_global_position_int(void);



#endif /* INC_MAV_MESSAGES_H_ */
