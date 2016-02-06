#ifndef ccmkv_h
#define ccmkv_h

#define FORKPIN             0  //PIN 62 =  PIN A8
#define FLIPPIN                   1  //PIN 63 =  PIN A9
#define THROTTLEPIN                2  //PIN 64 =  PIN A10
#define STEERPIN                   3  //PIN 65 =  PIN A11
#define AUX1PIN                    4  //PIN 66 =  PIN A12 s
#define AUX2PIN                    5  //PIN 67 =  PIN A13
#define AUX3PIN                    6  //PIN 68 =  PIN A14
#define AUX4PIN                    7  //PIN 69 =  PIN A15

#define FORKPOUT                   2
#define GRIPPOUT                   3
#define STEERPOUT                  5
#define THROTTLEPOUT               6
#define PILOTPOUT                  7
#define SPARKPOUT                  8
#define BLASTPOUT                  9

#define POTGND                     A5
#define POTPWR                     A3
#define POTSIG                     A1

#define POTMIN_SAFE              222
#define POTMAX_SAFE              778
#define FLOOR_HOME               750
//12:30 and 64:72

#define ABFSWITCH_THRESHOLD 1700
#define DEADBAND            60

#define pcnt2servoTime(x)  (x - -100) * (MAX_PULSE_WIDTH - MIN_PULSE_WIDTH) / (100 - -100) + MIN_PULSE_WIDTH
#define servoTime2pcnt(x) map(x, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH, 100,-100)
#endif

uint16_t deadband(uint16_t x, uint16_t band) {
  return abs((int)x - (int)1500) > band ? x : 1500;
}
