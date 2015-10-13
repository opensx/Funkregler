#include <EEPROM.h>

/** sxutils.cpp
 * contains utils to convert from bin to ascii string "payload"
 * which is used in xbee data transfer (API mode)
 * string always ends with a newline char 
 * and is always zero terminated
 */

#include "sxutils.h"
//#include "stdio.h" avoided because of 1.5k more ROM usage

char /* uint8_t */ payload[PAYLOAD_LEN]; // 12 byte string for sending data to coordinat
uint8_t lastAddr[N_ADDR];  // for storing the last 4 addresses
uint8_t addrUsage[N_ADDR];  // store usage of last 4 addresses
uint8_t indexLastAddr;
    
/**
add the 4 last address as indirect_address = 100 ...103 to the 
number of possible addresses.
for 100, return lastAddr[0] and so on
*/
uint8_t addressFunction(uint8_t aIn) {
    if ((aIn > MAX_ADDRESS) && 
        (aIn <= MAX_ADDRESS+N_ADDR) ) {
        return lastAddr[aIn-MAX_ADDRESS-1];
    } else {
        return aIn;
    }
}
        
/** build payload string from 1 number
 *  the channel number 1..255
 */
/*NOT USED void makePayload1Number(uint8_t d) {
   memset(payload,0,sizeof(payload));
   num2payload(0, (uint16_t)d);
} */


/** build payload string from 1 char and 2 numbers
 *  first is the channel number 1..99 (and 127)
 *  second is data 0..255
 */
void makePayloadCommandAnd2Numbers(char c, uint8_t ch, uint8_t d) {
   //snprintf(payload,PAYLOAD_LEN,"%u %u", ch ,d);  // avoided because of ROM size
   memset(payload,0,sizeof(payload));
   payload[0] = c;
   uint8_t end = num2payload(1, ch);
   if (end != 0) {
      payload[end-1] = ' '; // replace '\n' by ' '
      num2payload(end, (uint16_t)d); 
   }
}

/** build payload string from 1 char and 1 number
 *  first is command 'a' ...'z'
 *  second is data 0..999
 */
void makePayloadCommandAndNumber(char c, uint16_t d) {
   memset(payload,0,sizeof(payload));
   payload[0] = c;
   num2payload(1, d);
}

/** fill payload string with the number d, starting at
 * position "start" of payload string
 * return index of "0" in string (to be able to 
 * append here)
*/
uint8_t num2payload(uint8_t start, uint16_t d) {
   if (start >= (PAYLOAD_LEN-5)) return 0; // ERROR
   // convert number to string
   uint8_t p3 = d / 100;
   uint8_t p2 = (d - 100*p3) / 10;
   uint8_t p1 = (d - p3*100 - p2*10);
   if (p3 == 0) {
      if (p2 == 0) { 
        // only 1 digit
        payload[start]  = p1 + '0';
        payload[start+1]='\n';
        payload[start+2]= 0;  // string end
        return (start+2);
      } else {
        payload[start]  = p2 + '0';
        payload[start+1]= p1 + '0';
        payload[start+2]='\n';
        payload[start+3]= 0;  // string end
        return (start+3);
      }
   } else {
      payload[start]  = p3 + '0';
      payload[start+1]= p2 + '0';
      payload[start+2]= p1 + '0';
      payload[start+3]='\n';
      payload[start+4]= 0;  // string end
      return (start+5);
   }

}
