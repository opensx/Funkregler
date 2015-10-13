/* SXLoco.h
 * definitions for an Selectrix Loco
 *
 */
#ifndef SXLOCO_H_
#define SXLOCO_H_


#include "sxutils.h"



class SXLoco {
public:
	SXLoco();
	SXLoco(uint8_t);

    void toggleF0(void);
    uint8_t getF0(void);
    
    void toggleF1(void);
    uint8_t getF1(void);
    
    void updateSpeed(int8_t);  //with delta from rotary encode
    uint8_t getSpeed(void);
    uint8_t getBackward(void);
    void stop(void);    // set speed to zero, doesn't change dir
    
    void setIndex(uint8_t);
    uint8_t getIndex(void);
    uint8_t getAddress(void);

    uint8_t hasChanged(void); 
    void resetChanged(void);

    uint8_t getSXData(void);
    void setFromSXData(uint8_t);
    

private:
    void increaseSpeed(int8_t);
    void decreaseSpeed(int8_t);
    void increaseAddress(int8_t);
    void decreaseAddress(int8_t);
    
    uint8_t _index; // index into lastAddr[] table
    // address gets calculated from lastAddr[] table
    uint8_t _sxData = 0;  // contains ALL (current) loco data
	uint8_t _tmpSpeed =0 ;  // used in speed selection
	uint8_t _tmpBackward = 0;  // used in speed selection

    uint8_t _changed = 1;  // loco address has changed
    
};

#endif // SXLOCO_H_
