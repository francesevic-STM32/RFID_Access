/*
 * poruke.h
 *
 *  Created on: Feb 1, 2025
 *      Author: danijel.francesevic
 */

#ifndef INC_PORUKE_H_
#define INC_PORUKE_H_

#include <stdint.h>

// Denied messages
extern const char *denied_messages[][2];
extern const uint8_t denied_messages_count;

// Granted messages
extern const char *granted_messages[][2];
extern const uint8_t granted_messages_count;

// Welcome messages
extern const char *welcome_messages[][2];
extern const uint8_t welcome_messages_count;

#endif /* INC_PORUKE_H_ */
