/* pcb-type.h
 * definitions for the PCBoard
 *
 */
#ifndef PCB_TYPE_H_
#define PCB_TYPE_H_

//#define HW_REV  1.0        // pcb with teensy
//#define HW_REV  1.1p   // prototype with Arduino Pro Mini
#define HW_REV   "1.1"     // pcb 1.1 with Arduino Pro Mini

#define DISP_ROTATED    // if defined, rotate display by 180' and
                          // exchange "A" and "F" buttons



//#define _TEENSY_31     // default is now ATmega328 !
//#define _TEENSY_LC
//#define _DEBUG_AVR // if defined, the debut output is sent to the 
                   // AT-configured XBEE
// #define _DEBUG   // if debug => output to Serial Port ONLY FOR TEENSY !!


//******* HARDWARE PINS *****************************************************
#if defined(_TEENSY_31) || defined(_TEENSY_LC) // ***************************
#define ENC_PIN1   5
#define ENC_PIN2   6
#define ENC_BTN    7   // sets speed to zero
#define ADDR_BTN  19   // toogle between address selection and speed selection
#define F0_BTN    18
#define F1_BTN    17
#define PWR_DOWN  21   // to generate a power down interrupt
#define MEAS_VIN  A6
#define SLEEP_RQ  22

#else   // arduino pro mini *************************************************

#ifdef HW_REF=="HW1.1p"
#define ENC_PIN1   3
#define ENC_PIN2   2  // 
#define ENC_BTN    5  //  // sets speed to zero
#else
#define ENC_PIN1   3
#define ENC_PIN2   5  // for HW 1.1
#define ENC_BTN    2  // for HW 1.1 + switch on  // sets speed to zero
#endif

#ifndef DISP_ROTATED // *****************************************************
#define ADDR_BTN  6    // toogle between address selection and speed selection
#define F0_BTN    7
#define F1_BTN    8
#else   // also rename 'A' 'L' 'F' buttons
#define ADDR_BTN  8    // toogle between address selection and speed selection
#define F0_BTN    7
#define F1_BTN    6
#endif  // DISP_ROTATED

#define PWR_DISP  A2   // power out for oled display
#define SLEEP_RQ  A1   // let xbee sleep
#define MEAS_VIN  A0   // measure battery voltage
#endif // *******************************************************************



#endif //PCB_TYPE_H_
