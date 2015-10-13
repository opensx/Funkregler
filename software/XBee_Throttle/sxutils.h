/** sxutils.h
 * for filling the payload string with information
 *
 * (c) Michael Blank
 *
 * 03.10.2015 initial version
 */

#ifndef SXUTILS_H_
#define SXUTILS_H_

#include <Arduino.h>
#include <inttypes.h>

// SELECTRIX SPECIFIC
#define MAX_SPEED  31   // min_speed = 0 ! 
#define MIN_ADDRESS   1
#define MAX_ADDRESS   99
#define INVALID_UI8   255  // invalid uint8_t value


#define EEPROM_START   10   // store relative index (0..3) into current address
   // last used loco
#define N_ADDR      4    // store the last 4 addresses

#define PAYLOAD_LEN    12

// global variable for payload string
extern char /* uint8_t */ payload[PAYLOAD_LEN] ;   // 12 byte string for sending data to coordinator
extern uint8_t lastAddr[N_ADDR];  // for storing the last 4 addresses
extern uint8_t addrUsage[N_ADDR];  // store usage of last 4 addresses
extern uint8_t indexLastAddr;

uint8_t addressFunction(uint8_t addr);

//void makePayload1Number(uint8_t ch);

//void makePayload2Numbers(unsigned char ch, unsigned char d); //uint8_t ch, uint8_t d);

void makePayloadCommandAndNumber(char c, uint16_t d);

void makePayloadCommandAnd2Numbers(char c, uint8_t ch, uint8_t d);

uint8_t num2payload(uint8_t start, uint16_t d);

#endif  //SXUTILS_H_
