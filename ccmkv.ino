#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050.h"
#include "HMC5883L.h"
#include "LeanServo.h"
#include "PWM_capture.h"
#include "Sensors.h"
#include "ccmkv.h"

//mux should default to UC
//pwms should come from y
#define DEBUGDRIVE

enum controls { LEVER = 0, SW2, WHEEL, TRIGGER, LHSW, JOY1X, JOY2X, JOY2Y, AUX2, RHSW, JOY1Y};
String ctlNames[] =  {"LEVER", "SW2", "WHEEL", "TRIGGER", "LHSW", "JOY1X", "JOY2X", "JOY2Y", "AUX2", "RHSW", "JOY1Y"};

#define UPSIDEDOWN_THRESHOLD 8000
#define RIGHTSIDEUP_THRESHOLD -8000
enum STATES { RUN = 0, ABOUTFACE, IGNITE, FLIP};
enum DRIVES  { THROTTLE = 0, STEER, FORKS };
boolean tipped = false;
boolean cStates[] = {1, 0, 0, 0};
typedef struct ampflow {
  Servo drive;
  float driveVal = 0;
  float scale = 1;
  float offset = 0;

} AMPFLOW;

typedef struct ampflowServo {
  Servo drive;
  float driveVal = 0;
  float offset = 0;
  float pos = 0;
  float kp = 3.6;
  float ref = 0;
  float currentHome = POTMAX_SAFE;
  float customHome = FLOOR_HOME;

  /*float kd = 1 ;
  float ki = .008;
  float lastError = 0;
  float error = 0;
  double lastTime = 0;
  float errorAcc = 0;*/
} AMPFLOWSERVO;

#define RCS_ON 2000
#define RCS_OFF 1000
typedef struct rcSwitch {
  Servo drive;
  float switchVal = 0;
} RCSWITCH;


ampflow Throttle, Steer;
ampflowServo Forks;
rcSwitch Sparker, Blast, Pilot, Gripper;

volatile uint16_t rcValue[RC_CHANS] = {1502, 1502, 1502, 1502, 1502, 1502, 1502, 1502}; // interval [1000;2000]s;//
volatile float potFilter = 0;
volatile float filterCount = 0;
uint8_t state = 0;
long abfTimeDelta, abfLastTime = 0;
boolean abfSwitch, lastAbfSwitch, abfFlag = false;

void configureTimerInterrupts() {
  TCCR5B = 0;
  TCCR5A = 0;          // normal counting mode
  TCCR5B = _BV(CS50);//|CS50|CS52);     // set prescaler of 1024
  TCNT5 = 0;              // clear the timer count
  TIFR5 = _BV(OCF5A);     // clear any pending interrupts;
  OCR5A = 200;           // 100hz interrupt
  TIMSK5 =  _BV(OCIE5A) ; // enable the output compare interrupt
}

volatile uint8_t count = 0;
volatile uint8_t slowCount = 0;
ISR (TIMER5_COMPA_vect)
{
  count++;
  slowCount++;

}


void setup() {
  configureReceiver();
  configureTimerInterrupts();
  sensors_init();
  //set up forkPot
  pinMode(POTPWR, OUTPUT);
  pinMode(POTGND, OUTPUT);
  digitalWrite(POTPWR, HIGH);
  digitalWrite(POTGND, LOW);

  Serial.begin(115200);


  Throttle.drive.attach(THROTTLEPOUT);
  Steer.drive.attach(STEERPOUT);
  Forks.drive.attach(FORKPOUT);
  Sparker.drive.attach(SPARKPOUT);
  Pilot.drive.attach(PILOTPOUT);
  Blast.drive.attach(BLASTPOUT);
  Gripper.drive.attach(GRIPPOUT);


  Steer.scale = -1;
  if (rcValue[SW2] > ABFSWITCH_THRESHOLD) {
    abfSwitch = true;
    Throttle.scale = 1;
  }
  else {
    abfSwitch = false;
    lastAbfSwitch = abfSwitch;
    Throttle.scale = -1;
  }


  Serial.println("---------------------------------------");
}
void loop() {

  static uint16_t lm = 0;
  static int tipover_persistance = 20;
  static int macro_persistance = 8;
  static int release_persistance = 8;
  static int flipTime = 0;
  sensors_update();
  potFilter += analogRead(POTSIG);
  filterCount++;
  if (count >= 5) {  //50hz high priority loop TODO: fix this ugly hack
    count = 0;


    if ((rcValue[JOY1X] > 1650) && (rcValue[JOY1Y] > 1650))
      macro_persistance-- ;
    else
      macro_persistance++;

    if (macro_persistance > 8) {
      macro_persistance = 8;
      cStates[FLIP] = false;
    }
    else if (macro_persistance < 0) {
      macro_persistance = 0;
      if (cStates[FLIP] == false)
        flipTime = millis();
      cStates[FLIP] = true;


    }


    //handle the 4 rc switches
    if (abs((int)1500 - (int)rcValue[JOY2X]) > 250) //sparker
      Sparker.switchVal = RCS_ON;
    else
      Sparker.switchVal = RCS_OFF;

    if (abs((int)1500 - (int)rcValue[JOY2Y]) > 280) // blast
      Blast.switchVal = RCS_ON;
    else
      Blast.switchVal = RCS_OFF;

    if (rcValue[LHSW] > 1500) {  // gripper
      Gripper.switchVal = RCS_ON;
      release_persistance = 8;
    }


    else
      release_persistance--;
    if (release_persistance < 0)
      Gripper.switchVal = RCS_OFF;

    //fork home modes
    if ( (rcValue[RHSW] <= 1700) && (rcValue[RHSW] >= 1300) ) // middle pos / floor home
      Forks.currentHome = FLOOR_HOME;
    else if (rcValue[RHSW] < 1300)
      Forks.currentHome = POTMAX_SAFE;
    else if (rcValue[RHSW] > 1700)
      if ((Forks.currentHome ==  FLOOR_HOME) || (Forks.currentHome ==  POTMAX_SAFE))
        Forks.currentHome = Forks.pos;


    if (rcValue[AUX2] < 1500) //Pilot
      Pilot.switchVal = RCS_OFF;
    else
      Pilot.switchVal = RCS_ON;





    //steering and drive
    Throttle.driveVal = servoTime2pcnt(rcValue[TRIGGER]);
    Steer.driveVal = servoTime2pcnt(rcValue[WHEEL]);
    //fork control


    Forks.pos = potFilter / filterCount;
    potFilter = 0;
    filterCount = 0;

    if (cStates[FLIP]) {

      if (((millis() - flipTime) < 500) || (!tipped) ) {
        Gripper.switchVal = RCS_ON;
        Forks.ref = FLOOR_HOME;
      }
      else {
        Gripper.switchVal = RCS_OFF;
        Forks.ref = POTMIN_SAFE;
      }


    }
    else {


      //Full PID is ew for a 0 intertia system!
      Forks.ref = min(max(map(rcValue[LEVER], 900, 1700, POTMIN_SAFE, Forks.currentHome), POTMIN_SAFE), POTMAX_SAFE);
    }

    float kterm = Forks.kp * (Forks.ref - Forks.pos) + 1500 ;

    Forks.driveVal  = min(max( kterm, MIN_PULSE_WIDTH), MAX_PULSE_WIDTH);



    if (Forks.pos > POTMAX_SAFE) // only allow > 1500 width
      Forks.driveVal = min(1500, Forks.driveVal);
    if (Forks.pos < POTMIN_SAFE) // only allow > 1500 width
      Forks.driveVal = max(1500, Forks.driveVal);

    if (slowCount == 2) {
#ifdef DEBUGFORK
      Serial.print(tipped);
      if (cStates[FLIP])
        if (((millis() - flipTime) < 500) || (!tipped))
          Serial.print("Flippin0\t");
        else
          Serial.print("Flippin1\t");
          
        Serial.print(Gripper.switchVal);
        Serial.print("\t");
        Serial.print((millis() - flipTime));
      if (Forks.driveVal != 1500)
        Serial.print("*****");
      /*Serial.print("error: ");
      Serial.print(Forks.error);
      Serial.print("dterm: ");
      Serial.print(dterm);
      Serial.print("\titerm: ");
      Serial.print(iterm);
          Serial.print("\titerm2: ");
      Serial.print(dt);*/
      Serial.print("\thome: ");

      Serial.print (Forks.currentHome);

      Serial.print("\tkterm: ");

      Serial.print (kterm);
      Serial.print("\tdrive: ");
      Serial.print (Forks.driveVal);
      Serial.print("\tpos ");
      Serial.print(Forks.pos);
      Serial.print("\tRC ");
      Serial.print(rcValue[RHSW]);
      Serial.print("\tref ");
      Serial.println(Forks.ref);

#endif
    }




    if (rcValue[SW2] > ABFSWITCH_THRESHOLD)
      abfSwitch = true;
    else
      abfSwitch = false;
    if (abfSwitch != lastAbfSwitch) {
      abfFlag = true;
      lastAbfSwitch = abfSwitch;
      Throttle.scale *= -1;
      abfTimeDelta = millis() - abfLastTime;
      abfLastTime = millis();
    }



    //write rc vals to motors
    Throttle.drive.writeMicroseconds(deadband(pcnt2servoTime( Throttle.offset + Throttle.scale * Throttle.driveVal), DEADBAND ));
    Steer.drive.writeMicroseconds(deadband(pcnt2servoTime(Steer.offset + Steer.scale * Steer.driveVal), DEADBAND));
    Forks.drive.writeMicroseconds(deadband(Forks.driveVal, 5));
    Gripper.drive.writeMicroseconds(Gripper.switchVal);
    Sparker.drive.writeMicroseconds(Sparker.switchVal);
    Pilot.drive.writeMicroseconds(Pilot.switchVal);
    Blast.drive.writeMicroseconds(Blast.switchVal);
  }


  if (slowCount >= 12) { //~20hz loop
    //self riht the robot
    slowCount = 0;
    if (getAxis(az) > UPSIDEDOWN_THRESHOLD) // remember we are upsude down!!!!
      tipover_persistance--;
    else if (getAxis(az) < RIGHTSIDEUP_THRESHOLD)
      tipover_persistance++;
    if (tipover_persistance > 20) {
      tipover_persistance = 20;
      tipped = false;
    }
    if (tipover_persistance < -20) {
      tipover_persistance = -20;
      tipped = true;
    }


    // do a 180 if we've requested it
    static float targetHeading = 0;
    if (abfFlag) {
      abfFlag = false;
      if (! cStates[ABOUTFACE]) {
        float initialHeading = atan2(getAxis(mx), getAxis(my));
        targetHeading = initialHeading + M_PI;
        cStates[ABOUTFACE] = true;
        Serial.println("about face!");
      }
    }

    if (cStates[ABOUTFACE]) {

      /* float heading = atan2(getAxis(mx), getAxis(my));
       if (heading < 0)
         heading += 2 * M_PI;
       //bang bang with .2 rad error cutoff , may want to try other options
       //abs(heading - targetHeading) <= .1 || (*/
      if ((millis() - abfLastTime) > 850) {
        cStates[ABOUTFACE] = false;
        Steer.offset = 0;
      }
      else
        Steer.offset = -40;
    }
  }

  if (slowCount == 2) {



#ifdef DEBUG0
    Serial.print( "drive:  " );//
    Serial.print( Forks.drive.readMicroseconds());
    Serial.print( "\tdriveVal:  " );//
    Serial.print( Forks.driveVal);//
    Serial.print( "\tpos " );//
    Serial.print( Forks.pos);//drive.readMicroseconds());
    Serial.print("\tref ");
    Serial.print(Forks.ref);
    Serial.print(" \t lever ");
    Serial.println(rcValue[LEVER]);
    //  Serial.println(millis() - lm);
    //  lm = millis();*/


#endif

#ifdef DEBUGDRIVE
  //  if (Throttle.drive.readMicroseconds() != 1500) {
      Serial.println( Throttle.driveVal);
      Serial.println(Throttle.drive.readMicroseconds());
      Serial.println( Steer.driveVal);
      Serial.println(Steer.drive.readMicroseconds());
      Serial.println(map( Throttle.scale * Throttle.driveVal, -100, 100, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH));
      Serial.println(rcValue[TRIGGER]);
      Serial.println();
      Serial.println();
 //   }
#endif

    //  both the imu and mag have a slow filter , but additional filtering maybe needed. testing on the todo
#ifdef DEBUGRC
    for (int i = 0 ; i < RC_CHANS; i++) {
      Serial.print(" " );
      Serial.print (ctlNames[i]) ;

      Serial.print (") ");
      Serial.print (rcValue[i]);
    }
    Serial.println();

#endif

  }
}
