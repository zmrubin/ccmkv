#include "PWM_capture.h"
void configureReceiver() {
  /******************    Configure each rc pin/port for PCINT    ***************************/
  DDRK = 0;  // defined PORTK as a digital port ([A8-A15] are consired as digital PINs and not analogical)
  DDRB &=  ~(0x70); // do the same for a select number of PORTB pins
  // PCINT activation

  PORTK |= 0xFF;
  PORTB |= 0x70;

  PCMSK2 |= 0xFF;
  PCMSK0 |= 0x70;
  PCICR = 0x05; //enable pin change interrupts 0 and 2

}


volatile uint16_t rcValue[RC_CHANS] = {1502, 1502, 1502, 1502, 1502, 1502, 1502, 1502}; // interval [1000;2000]



ISR(PCINT2_vect) { //this ISR handles the RX461 and 4 of the 92824 channels
  whichPin(2);
}

ISR(PCINT0_vect) { //this ISR handles 3 of the 92824 channels
  whichPin(0);
}

void whichPin(uint8_t pci_num) {
  uint8_t mask;
  uint8_t pin;
  uint16_t cTime, dTime;
  static uint16_t edgeTime[RC_CHANS];
  static uint8_t PCintLast[3] = {PINB, PINJ, PINK};

  pin = RX_PCINT_PIN_PORTS(pci_num); // pin state bitfield

  mask = pin ^ PCintLast[pci_num];   // doing a ^ between the current interruption and the last one indicates wich pin changed
  cTime = micros();         // micros() return a uint32_t, but it is not usefull to keep the whole bits => we keep only 16 bits
  PCintLast[pci_num] = pin;          // we memorize the current state of all PINs [D0-D7]
  sei();                    // re enable other interrupts at this point, the rest of this interrupt is not so time critical and can be interrupted safely
  //a bit of  a hack to keep the isr light
  int a = 0;
  int b = 8;
  int chan_begin = 0;
  if (pci_num == 0) {
    a = 4;
    chan_begin = b - a ;
    b = 7;
  }

  for (int i = a; i < b; i++) {
    if (mask & (1 << i)) {
      if (!(pin & (1 << i))) {
        dTime = cTime - edgeTime[i + chan_begin];
        if (900 < dTime && dTime < 2200) {
          rcValue[i + chan_begin] = dTime;
        }
      } else edgeTime[i + chan_begin] = cTime;
    }
  }


}

void setup() {
  configureReceiver();
  Serial.begin(115200);
  Serial.println("---------------------------------------");
}

void loop() {

  for (int i = 0 ; i < RC_CHANS; i++) {
    Serial.print(" " );
    Serial.print (i) ;

    Serial.print (") ");
    Serial.print (rcValue[i]);
  }
  delay(200);
  Serial.println();
}
