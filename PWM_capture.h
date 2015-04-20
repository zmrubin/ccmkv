#include "Stdint.h"
#define RC_CHANS 11

enum controls { LEVER=0, SW2, TRIGGER, WHEEL, LHSW, JOY1X, JOY2X, JOY2Y, AUX2, RHSW, JOY1Y};
 
#define RX_PCINT_PIN_PORTS(x)       ((x) == 0 ? (PINB) : ((x) == 1 ? (PINJ) : (PINK)) )


