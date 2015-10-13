/* SXLoco.cpp
 * contains all details of a selectrix(SX1) loco
 
 * cannot include "EEPROM.h" here:
 * "This seems to be a general problem of the Arduino IDE: 
 * It only recognizes libraries that are included in the 
 * (primary) .ino file.
 * solution: include EEPROM.h only in .ino file "
 */

#include "sxutils.h"
#include "SXLoco.h"

SXLoco::SXLoco() {
    setIndex(0);
    _sxData = 0;
    _changed= 1;
}

SXLoco::SXLoco(uint8_t i) {
    setIndex(i);
    _sxData = 0;
    _changed= 1;
}

uint8_t SXLoco::hasChanged() {
   return _changed;
}

void SXLoco::resetChanged() {
   _changed = 0;
}

void SXLoco::stop() {
    _sxData &=0xE0; // set speed bits (only) to zero
    _changed=1;
}

uint8_t SXLoco::getSpeed() {
    return _sxData & 0x1F;
}

void SXLoco::toggleF0() {
   if (bitRead(_sxData,6)) {
      bitClear(_sxData,6);
   } else {
      bitSet(_sxData,6);
   }
   _changed=1;
}

uint8_t SXLoco::getF0() {
   return bitRead(_sxData,6);
}

void SXLoco::toggleF1() {
   if (bitRead(_sxData,7)) {
      bitClear(_sxData,7);
   } else {
      bitSet(_sxData,7);
   }
   _changed=1;
}

uint8_t SXLoco::getF1() {
   return bitRead(_sxData,7);
}

uint8_t SXLoco::getBackward() {
    return bitRead(_sxData,5);
}

uint8_t SXLoco::getAddress() {
    return lastAddr[_index];
}

void SXLoco::setIndex(uint8_t i) {
    if ((i >=0 ) && (i < N_ADDR)) {
       _index = i;
       _changed=1;
    }
}

uint8_t SXLoco::getSXData() {
   // sx data (bit 5=direction, bit 6= light, bit7= horn)
   return _sxData;
}

void SXLoco::setFromSXData(uint8_t data) {
   // sx data (bit 5=direction, bit 6= light, bit7= horn)
   _sxData = data;
}

/** update SXLoco speed from encoder "delta"
 * larger delta => make larger speed steps
 */
void SXLoco::updateSpeed(int8_t delta) {
// Serial.print("delta=");
// Serial.println(delta);

   _tmpSpeed = _sxData & 0x1F;
   _tmpBackward = bitRead(_sxData,5);
   if (_tmpSpeed == 0) {     
        if (_tmpBackward) {
          if (delta > 0) {
            // then only change direction if delta > 0
            _tmpBackward = 0;
          } else {
             increaseSpeed(delta);
          }  
        } else { // forwards
          if (delta < 0) {
            // then only change direction if delta > 0
            _tmpBackward = 1;
          } else {
            increaseSpeed(delta);
          }  
        }
          
      } else {  // speed is not == 0
        if (_tmpBackward) {
          if (delta > 0) {
            if (_tmpSpeed > 3) { // do only if not close to ZERO
                decreaseSpeed(delta);
            } else {
                decreaseSpeed(1);
            }
          } else {
            increaseSpeed(delta);
          }
        } else {
          if (delta > 0) {
            increaseSpeed(delta);
          } else {
            if (_tmpSpeed > 3) { // do only if not close to ZERO
                decreaseSpeed(delta);
            } else {
                decreaseSpeed(1);
            }
          }
        }         
      }


   // did speed or direction change ?
   if ( (_tmpSpeed != (_sxData & 0x1F)) ||
        (_tmpBackward != bitRead(_sxData,5)) ) {
       _sxData = (_sxData & 0xE0) | _tmpSpeed;
       if (_tmpBackward) {
           bitSet(_sxData,5);
       } else {
           bitClear(_sxData,5);
       }
       _changed=1;   
   }
}

/** decrease by 1 step if abs(ad) == 1
     by 2 steps if abs(ad) >=2
     and by 3 steps if abs(ad) >=4
     */
void SXLoco::decreaseSpeed(int8_t ad) {
    ad = abs(ad);
    if (_tmpSpeed > 0)  {
       // decrease speed
      _tmpSpeed--;
       if ((_tmpSpeed > 0) && (ad > 1)) {
          _tmpSpeed--;
          if ((_tmpSpeed > 0) && (ad > 3)){
          _tmpSpeed--;
          }
       }
   } 
}

/** inccrease by 1 step if abs(ad) == 1
     by 2 steps if abs(ad) >=2
     and by 3 steps if abs(ad) >=4
*/
void SXLoco::increaseSpeed(int8_t ad) {
    ad = abs(ad);
    if (_tmpSpeed < MAX_SPEED)  {
       // increase speed
      _tmpSpeed++;
       if ((_tmpSpeed < MAX_SPEED) && (ad > 1)) {
          _tmpSpeed++;
          if ((_tmpSpeed < MAX_SPEED) && (ad > 3)) {
          _tmpSpeed++;
          }
       }
   } 
}

