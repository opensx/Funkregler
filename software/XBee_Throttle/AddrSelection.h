/* AddrSelection.h

*/

#ifndef ADDR_SELECTION_H_
#define ADDR_SELECTION_H_

#include <Arduino.h>
#include <inttypes.h>
#include <EEPROM.h>
#include "sxutils.h"

class AddrSelection {
public:
   AddrSelection();
   void initFromEEPROM(void);
   void init(uint8_t addr); 
   uint8_t finish(void);
   uint8_t getCurrentAddress();
   uint8_t getCurrentStoredIndex();
   void update(int8_t delta);  // update from rotary encoder delta

private:
    uint8_t _selectedAddress;
    uint8_t _oldAddress;
    void increase(int8_t d);
    void decrease(int8_t d);
};

#endif // ADDRESS_SELECTION_H_