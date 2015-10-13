#Software für den Funkregler selber, HW siehe:

<a href="http://opensx.net/funkregler"> Funkregler auf opensx </a>

#XBee_Throttle.ino

version for Teensy3.1, TeensyLC or ATmega328

for teensy's:
  XBee is connected to RX1 (pin2) and TX1 (pin3)

for avr/atmega328
  XBee is connected to RX (pin0) and TX (pin1)

SSD1306 display 128x64 - weil sie weniger RAM braucht als die 
ADAfruit lib, wird die U8glib benutzt.
genutzte Fonts: 
   u8g_font_fur42n
   u8g_font_fur20
   u8g_font_helvB12  

Rotary Encoder für Speed und Adress Selection
Buttons für Adress-Selection ("A"), Licht (F0="L") und Function (F1="F")

13.10.2015 "waiting_for_response" modus hinzugefügt
    (nach: neue Lok selektiert, vor: noch keine Antwort von der Zentrale)

12.10.2015 simplified, removed settings, protocol rev.1.1.

5.10.2015 using SXLoco instead of "general" Loco
   larger encoder rotation => increase/decr address by several steps
2.10.2015 distinction between 0-forward and 0-backward
   larger encoder rotation => increase/decr speed by several steps

31.8.2015 - SELECTRIX MODE
4.9.2015 - bidirectional communication of trackPower
(press RotEnc Button for longer than 1.3 secs to toggle power)
5.9.2015 - "Batt" is displayed when battery below 3300 mV
   (and battery state [10mV] is sent every 10mins to coord: "B 352" )

(c) Michael Blank - 2015 - opensx.net

#Libraries needed
the following libraries are used - make sure you have a version matching to
your hardware, either for the
-  "Arduino Pro Mini" (ATMega328)
or for the
- "teensy", see https://www.pjrc.com/teensy/teensy31.html

u8glib
Encoder
Bounce
XBee

(+ SX22h, SXCommand and sxutils, SXLoco, AddrSelect from this website)
A
C
A
 
