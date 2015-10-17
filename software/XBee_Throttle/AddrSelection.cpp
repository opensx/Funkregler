/* AddrSelection.cpp

*/


#include "AddrSelection.h"

/**EEPROM Usage:
   start:  index to last used addr (0...3)  =EEPROM_START
   start+1:  adr[0]
   start+2:  adr[1]
   start+3:  adr[2]
   start+4:  adr[3] 
*/

AddrSelection::AddrSelection() {
    ;
}

void AddrSelection::initFromEEPROM() {
   // get loco address to start with
   uint8_t i;
   indexLastAddr = EEPROM.read(EEPROM_START);  // index to last used loco
   if ((indexLastAddr < 0) || (indexLastAddr >= N_ADDR)) { // no useful data, init EEPROM
       indexLastAddr=0;
       EEPROM.write(EEPROM_START,indexLastAddr);
       for (i=0; i <N_ADDR; i++) {
          lastAddr[i]=50+i;
          EEPROM.write(EEPROM_START+1+i,lastAddr[i]);
          addrUsage[i] = 0;
       }
   } else {  // there are addresses stored in EEPROM
       for (i=0; i <N_ADDR; i++) {
          lastAddr[i]=EEPROM.read(EEPROM_START+1+i);
          addrUsage[i] = 1;
       }
   }
}


void AddrSelection::init(uint8_t index) {   
    _selectedAddress = MAX_ADDRESS+1+index;  //pseudo addr 100..103
    _oldAddress = lastAddr[index];  // real SX Address
}
              
uint8_t AddrSelection::finish() {
    uint8_t addr = addressFunction(_selectedAddress);

    // get sxdata of the new address
    // TODO
    if (_oldAddress != addr) {
        // if this address is not already contained in "FIRST-4" 
             // addresses, add it to the "lastAdr[]" list
             uint8_t i, found=0;
             for (i=0; i < N_ADDR; i++) {
                 if (lastAddr[i] == addr) {
                     found = 1;
                     indexLastAddr = i; // store index to current address
                     EEPROM.write(EEPROM_START, indexLastAddr);
                     if (addrUsage[i] < 255) addrUsage[i]++;
                     break;
                 }
             }
             if (found == 0) {  // store new address in EEPROM
                // find least used address
                uint8_t i, min=255;
                for (i=0; i<N_ADDR; i++) {
                    if (addrUsage[i] < min) min=addrUsage[i];
                }
                for (i=0; i<N_ADDR; i++) {
                    if (addrUsage[i] == min) {
                        indexLastAddr = i;  // use this index for new storage
                    }
                }
                lastAddr[indexLastAddr] = addr;
                addrUsage[indexLastAddr] = 1;
                EEPROM.write(EEPROM_START + 1+ indexLastAddr, addr);
                EEPROM.write(EEPROM_START, indexLastAddr);
             }
        return 1;  // address has changed
    }
    return 0;  // address has not changed
}

/** real address of the currently selected address
*/
uint8_t AddrSelection::getCurrentAddress() {
    return addressFunction(_selectedAddress);  //real address
}

/** is the currently selected address one of the
 * previously stored addresses? then return its index
 * starting from 1...4 (for display)
 */
uint8_t AddrSelection::getCurrentStoredIndex() {
   if (_selectedAddress <= MAX_ADDRESS) {
      return INVALID_UI8;  // == this is a real address
   } else {
      return _selectedAddress-MAX_ADDRESS; // index 1,2,3.4 of #store
   }
}
   
              
/** update address from encoder "delta"
 * larger delta => make larger steps
 */
void AddrSelection::update(int8_t delta) {
// Serial.print("delta=");
// Serial.println(delta); 
   if (delta > 0) {
       increase(delta);
   } else {
       decrease(-delta);
   }
}

/** decrease by 1 step if abs(ad) == 1
  by 2 steps if abs(ad) >=2
  and by 3 steps if abs(ad) >=4
*/
void AddrSelection::decrease(int8_t ad) {
    _selectedAddress--;
    // for address in 100..103 range only decr by 1
    if (_selectedAddress <= MAX_ADDRESS) {
    if (ad > 1) {
      _selectedAddress-=2;
       if (ad > 3) {
          _selectedAddress-=3;
       }
    } 
    }
    // readjust address range
    if ((_selectedAddress < 1) || 
        (_selectedAddress > MAX_ADDRESS+N_ADDR)) {  // >103
        _selectedAddress = MAX_ADDRESS+N_ADDR;  // 103
    }
}

void AddrSelection::increase(int8_t ad) {
    _selectedAddress++;
    // for address in 100..103 range only incr by 1
    if(_selectedAddress < MAX_ADDRESS-2) {       
      if (ad > 1) {
        _selectedAddress+=2;
        if (ad > 3) {
          _selectedAddress+=3;
        }
      } 
    }
    // readjust
    if ((_selectedAddress < 1) || 
        (_selectedAddress > MAX_ADDRESS+N_ADDR)) {
        _selectedAddress = 1;
    }
}
