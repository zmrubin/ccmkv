// ************************************************************************************************************
// I2C Compass HMC5883
// ************************************************************************************************************
// I2C adress: 0x3C (8bit)   0x1E (7bit)
// ************************************************************************************************************
#include "Sensors.h"
imu_t imu;
global_conf_t global_conf;
bool calibrate_mag = true ;
uint16_t i2c_errors_count;

#define HMC58X3_R_CONFA 0
#define HMC58X3_R_CONFB 1
#define HMC58X3_R_MODE 2
#define HMC58X3_X_SELF_TEST_GAUSS (+1.16)                       //!< X axis level when bias current is applied.
#define HMC58X3_Y_SELF_TEST_GAUSS (+1.16)   //!< Y axis level when bias current is applied.
#define HMC58X3_Z_SELF_TEST_GAUSS (+1.08)                       //!< Y axis level when bias current is applied.
#define SELF_TEST_LOW_LIMIT  (243.0/390.0)   //!< Low limit when gain is 5.
#define SELF_TEST_HIGH_LIMIT (575.0/390.0)   //!< High limit when gain is 5.
#define HMC_POS_BIAS 1
#define HMC_NEG_BIAS 2

#define MAG_ADDRESS 0x1E
#define MAG_DATA_REGISTER 0x03

static uint8_t rawADC[6];
static int32_t xyz_total[3] = {0, 0, 0}; // 32 bit totals so they won't overflow.


void i2c_rep_start(uint8_t address) {
  waitTransmissionI2C((1 << TWINT) | (1 << TWSTA) | (1 << TWEN)); // send REPEAT START condition and wait until transmission completed
  TWDR = address;                                           // send device address
  waitTransmissionI2C((1 << TWINT) | (1 << TWEN));          // wail until transmission completed
}

void i2c_stop(void) {
  TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWSTO);
  //  while(TWCR & (1<<TWSTO));                // <- can produce a blocking state with some WMP clones
}

void i2c_write(uint8_t data ) {
  TWDR = data;                                 // send data to the previously addressed device
  waitTransmissionI2C((1 << TWINT) | (1 << TWEN));
}

uint8_t i2c_readAck() {
  waitTransmissionI2C((1 << TWINT) | (1 << TWEN) | (1 << TWEA));
  return TWDR;
}

uint8_t i2c_readNak() {
  waitTransmissionI2C((1 << TWINT) | (1 << TWEN));
  uint8_t r = TWDR;
  i2c_stop();
  return r;
}

void i2c_read_reg_to_buf(uint8_t add, uint8_t reg, uint8_t *buf, uint8_t size) {
  i2c_rep_start(add << 1); // I2C write direction
  i2c_write(reg);        // register selection
  i2c_rep_start((add << 1) | 1); // I2C read direction
  uint8_t *b = buf;
  while (--size) *b++ = i2c_readAck(); // acknowledge all but the final byte
  *b = i2c_readNak();
}

void i2c_getSixRawADC(uint8_t add, uint8_t reg) {
  i2c_read_reg_to_buf(add, reg, rawADC, 6);
}

void i2c_writeReg(uint8_t add, uint8_t reg, uint8_t val) {
  i2c_rep_start(add << 1); // I2C write direction
  i2c_write(reg);        // register selection
  i2c_write(val);        // value to write in register
  i2c_stop();
}

uint8_t i2c_readReg(uint8_t add, uint8_t reg) {
  uint8_t val;
  i2c_read_reg_to_buf(add, reg, &val, 1);
  return val;
}

static void getADC() {
  i2c_getSixRawADC(MAG_ADDRESS, MAG_DATA_REGISTER);
  MAG_ORIENTATION( ((rawADC[0] << 8) | rawADC[1]) ,
                   ((rawADC[4] << 8) | rawADC[5]) ,
                   ((rawADC[2] << 8) | rawADC[3]) );
}



void __attribute__ ((noinline)) waitTransmissionI2C(uint8_t twcr) {
  TWCR = twcr;
  uint8_t count = 255;
  while (!(TWCR & (1 << TWINT))) {
    count--;
    if (count == 0) {            //we are in a blocking state => we don't insist
      TWCR = 0;                  //and we force a reset on TWINT register
#if defined(WMP)
      neutralizeTime = micros(); //we take a timestamp here to neutralize the value during a short delay
#endif
      i2c_errors_count++;
      break;
    }
  }
}



static uint8_t bias_collect(uint8_t bias) {
  int16_t abs_magADC;

  i2c_writeReg(MAG_ADDRESS, HMC58X3_R_CONFA, bias);            // Reg A DOR=0x010 + MS1,MS0 set to pos or negative bias
  for (uint8_t i = 0; i < 10; i++) {                           // Collect 10 samples
    i2c_writeReg(MAG_ADDRESS, HMC58X3_R_MODE, 1);
    delay(100);
    getADC();                                                  // Get the raw values in case the scales have already been changed.
    for (uint8_t axis = 0; axis < 3; axis++) {
      abs_magADC =  abs(imu.magADC[axis]);
      xyz_total[axis] += abs_magADC;                           // Since the measurements are noisy, they should be averaged rather than taking the max.
      if ((int16_t)(1 << 12) < abs_magADC) return false;       // Detect saturation.   if false Breaks out of the for loop.  No sense in continuing if we saturated.
    }
  }
  return true;
}

 void Mag_init() {
   
     i2c_writeReg(MPU6050_ADDRESS, 0x1C, 0x10);             //ACCEL_CONFIG  -- AFS_SEL=2 (Full Scale = +/-8G)  ; ACCELL_HPF=0   //note something is wrong in the spec.
  //note: something seems to be wrong in the spec here. With AFS=2 1G = 4096 but according to my measurement: 1G=2048 (and 2048/8 = 256)
  //confirmed here: http://www.multiwii.com/forum/viewtopic.php?f=8&t=1080&start=10#p7480

  #if defined(MPU6050_I2C_AUX_MASTER)
    //at this stage, the MAG is configured via the original MAG init function in I2C bypass mode
    //now we configure MPU as a I2C Master device to handle the MAG via the I2C AUX port (done here for HMC5883)
    i2c_writeReg(MPU6050_ADDRESS, 0x6A, 0b00100000);       //USER_CTRL     -- DMP_EN=0 ; FIFO_EN=0 ; I2C_MST_EN=1 (I2C master mode) ; I2C_IF_DIS=0 ; FIFO_RESET=0 ; I2C_MST_RESET=0 ; SIG_COND_RESET=0
    i2c_writeReg(MPU6050_ADDRESS, 0x37, 0x00);             //INT_PIN_CFG   -- INT_LEVEL=0 ; INT_OPEN=0 ; LATCH_INT_EN=0 ; INT_RD_CLEAR=0 ; FSYNC_INT_LEVEL=0 ; FSYNC_INT_EN=0 ; I2C_BYPASS_EN=0 ; CLKOUT_EN=0
    i2c_writeReg(MPU6050_ADDRESS, 0x24, 0x0D);             //I2C_MST_CTRL  -- MULT_MST_EN=0 ; WAIT_FOR_ES=0 ; SLV_3_FIFO_EN=0 ; I2C_MST_P_NSR=0 ; I2C_MST_CLK=13 (I2C slave speed bus = 400kHz)
    i2c_writeReg(MPU6050_ADDRESS, 0x25, 0x80|MAG_ADDRESS);//I2C_SLV0_ADDR -- I2C_SLV4_RW=1 (read operation) ; I2C_SLV4_ADDR=MAG_ADDRESS
    i2c_writeReg(MPU6050_ADDRESS, 0x26, MAG_DATA_REGISTER);//I2C_SLV0_REG  -- 6 data bytes of MAG are stored in 6 registers. First register address is MAG_DATA_REGISTER
    i2c_writeReg(MPU6050_ADDRESS, 0x27, 0x86);             //I2C_SLV0_CTRL -- I2C_SLV0_EN=1 ; I2C_SLV0_BYTE_SW=0 ; I2C_SLV0_REG_DIS=0 ; I2C_SLV0_GRP=0 ; I2C_SLV0_LEN=3 (3x2 bytes)
  #endif
  
  bool bret = true;              // Error indicator

  // Note that the  very first measurement after a gain change maintains the same gain as the previous setting.
  // The new gain setting is effective from the second measurement and on.
  i2c_writeReg(MAG_ADDRESS, HMC58X3_R_CONFB, 2 << 5);  //Set the Gain
  i2c_writeReg(MAG_ADDRESS, HMC58X3_R_MODE, 1);
  delay(100);
  getADC();  //Get one sample, and discard it

  if (!bias_collect(0x010 + HMC_POS_BIAS)) bret = false;
  if (!bias_collect(0x010 + HMC_NEG_BIAS)) bret = false;

  if (bret) // only if no saturation detected, compute the gain. otherwise, the default 1.0 is used
    for (uint8_t axis = 0; axis < 3; axis++)
      magGain[axis] = 820.0 * HMC58X3_X_SELF_TEST_GAUSS * 2.0 * 10.0 / xyz_total[axis]; // note: xyz_total[axis] is always positive

  // leave test mode
  i2c_writeReg(MAG_ADDRESS , HMC58X3_R_CONFA , 0x70 ); //Configuration Register A  -- 0 11 100 00  num samples: 8 ; output rate: 15Hz ; normal measurement mode
  i2c_writeReg(MAG_ADDRESS , HMC58X3_R_CONFB , 0x20 ); //Configuration Register B  -- 001 00000    configuration gain 1.3Ga
  i2c_writeReg(MAG_ADDRESS , HMC58X3_R_MODE  , 0x00 ); //Mode register             -- 000000 00    continuous Conversion Mode
  delay(100);
}


static void Device_Mag_getADC() {
  getADC();
}
uint8_t Mag_getADC() { // return 1 when news values are available, 0 otherwise
  static uint32_t t, tCal = 0;
  static int16_t magZeroTempMin[3], magZeroTempMax[3];
  uint8_t axis;

  // if ( currentTime < t ) return 0; //TODO: INSERT ACTUAL RR TIMER CHECK HERE
  //t = currentTime + 100000;
  Device_Mag_getADC();

  for (axis = 0; axis < 3; axis++) {
    imu.magADC[axis]  = imu.magADC[axis]  * magGain[axis];
    if (!calibrate_mag) imu.magADC[axis]  -= global_conf.magZero[axis];
  }

  if (calibrate_mag) {
    if (tCal == 0) // init mag calibration
      tCal = t;
    if ((t - tCal) < 30000000) { // 30s: you have 30s to turn the multi in all directions
      LEDPIN_TOGGLE;
      for (axis = 0; axis < 3; axis++) {
        if (tCal == t) { // it happens only in the first step, initialize the zero
          magZeroTempMin[axis] = imu.magADC[axis];
          magZeroTempMax[axis] = imu.magADC[axis];
        }
        if (imu.magADC[axis] < magZeroTempMin[axis]) {
          magZeroTempMin[axis] = imu.magADC[axis];
        } //SET_ALARM(ALRM_FAC_TOGGLE, ALRM_LVL_TOGGLE_1);}
        if (imu.magADC[axis] > magZeroTempMax[axis]) {
          magZeroTempMax[axis] = imu.magADC[axis];
        } //SET_ALARM(ALRM_FAC_TOGGLE, ALRM_LVL_TOGGLE_1);}
        global_conf.magZero[axis] = (magZeroTempMin[axis] + magZeroTempMax[axis]) >> 1;
      }
    } else {
      calibrate_mag = 0;
      tCal = 0;
      // writeGlobalSet(1);
    }
  }

#if defined(SENSORS_TILT_45DEG_LEFT)
  int16_t temp = ((imu.magADC[PITCH] - imu.magADC[ROLL] ) * 7) / 10;
  imu.magADC[ROLL] = ((imu.magADC[ROLL]  + imu.magADC[PITCH]) * 7) / 10;
  imu.magADC[PITCH] = temp;
#endif
#if defined(SENSORS_TILT_45DEG_RIGHT)
  int16_t temp = ((imu.magADC[PITCH] + imu.magADC[ROLL] ) * 7) / 10;
  imu.magADC[ROLL] = ((imu.magADC[ROLL]  - imu.magADC[PITCH]) * 7) / 10;
  imu.magADC[PITCH] = temp;
#endif

  return 1;
}


