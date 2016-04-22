/* 
  PID for Roll and Pitch Axis
  QC Project EWU 2016
  Allows for bluetooth control of testing PID parameters of motors

  Revision 0: created 
  Revision 1: Added "Motors" class
  Revision 2: Made PID parameters variables to be controlled remotely


*/

#include <PID_v1.h>
#include <i2c_t3.h>
#include <XYZ_BNO055.h>
#include <Motors.h>

#define BT Serial
//#define BT Serial2
#define TimeIntervalMilliSeconds 10  // Milliseconds 100 Hz
#define ms2us 1000 // us per ms 
#define u2deltaPWM_R 1
#define u2deltaPWM_P 1
#define TRUE 1 
#define FALSE 0
#define PID_MAX_VALUE 100
#define bitRes 65535
#define LEDPIN 13

float KiP = 0, KiR = 0, KpP = .25, KpR = .25, KdP = 0, KdR = 0 ;
double consKpP, consKdP, consKiP, InputP, OutputP, SetpointP;
double consKpR, consKdR, consKiR, InputR, OutputR, SetpointR;
float basePWM = 0 ;
float ypr[3] ;
float rollOffset, pitchOffset ;
float Roll_Setpoint = 0, Pitch_Setpoint = 0;
float yawOffset = 0 ;
volatile int newData = 0 ;

XYZ_BNO055 imu ;
PID PID_P(&InputP, &OutputP, &SetpointP, consKpP, consKiP, consKdP, DIRECT);
PID PID_R(&InputR, &OutputR, &SetpointR, consKpR, consKiR, consKdR, DIRECT);
Motors motorControl ;

IntervalTimer IMUUpdate ;

void setup() {
  /*Declarations*/
  char go ;
  pinMode(LEDPIN, OUTPUT);
  motorControl.initMotors() ;
  
  /* Setup The BNO055 */
  Wire.begin(I2C_MASTER, 0x00, I2C_PINS_16_17, I2C_PULLUP_EXT, I2C_RATE_400);
  delay(150) ;
  Serial.begin(115200) ;
  delay(150) ;
  Serial.println("BNO055 TEST");
  while (!imu.setup(BNO055_ADDRESS_B))
  {
    Serial.println("No BNO055 found");
    delay(100);
  }
  Serial.println("BNO055 found") ;
  delay(1000) ;
  imu.setMode(XYZ_BNO055::NDOF) ;
  calibrateIMU();
  Serial.println("IMU Calibration Success");
  delay(1000);
  digitalWrite(LEDPIN,HIGH);

  /* Setup Bluetooth */
  BT.begin(115200) ;

  /* Setup The Motors */
  // Takes ~20 seconds
  //basePWM = calibrateESCs() ;
  basePWM = 40.0 ;
  
  /* Setup The PID */
  //Pitch
  consKpP = KpP;
  consKiP = KiP;
  consKdP = KdP;
  SetpointP = Pitch_Setpoint ;
  PID_P.SetMode(AUTOMATIC);
  PID_P.SetOutputLimits(-PID_MAX_VALUE, PID_MAX_VALUE);
  PID_P.SetSampleTime(TimeIntervalMilliSeconds);
  PID_P.SetTunings(consKpP, consKiP, consKdP);

  //Roll
  consKpR = KpR;
  consKiR = KiR;
  consKdR = KdR;
  SetpointR = Roll_Setpoint ;
  PID_R.SetMode(AUTOMATIC);
  PID_R.SetOutputLimits(-PID_MAX_VALUE, PID_MAX_VALUE);
  PID_R.SetSampleTime(TimeIntervalMilliSeconds);
  PID_R.SetTunings(consKpR, consKiR, consKdR);

  /* PWM Setup */
  analogWriteResolution(16) ;
  
  /* Get ready to start */
  BT.println("'Start to Begin'");
  while(true) {
    if(BT.available()) {
        go = (char)BT.read() ;
        if (go == 'g')
          break ;
    }  
    Serial.println("waiting for 'g'...");
    
    delay(1000);
  }

  /* Timer Interrupt Setup */
  // Interrupt is armed after this call
  IMUUpdate.begin(imuISR, TimeIntervalMilliSeconds * ms2us) ;
}

void imuISR ( void ) {

  imu.readYPR(ypr) ;
  newData = TRUE ;
  //Serial.println("IMU READ!!!");
}

void calibrateIMU() {
  Serial.println("Cal: No=0, full=3");

  uint8_t stats[4];
  for (int i = 0; i < 60; i++) {
    imu.readCalibration(stats);
    if ((stats[0] == 3) && (stats[1] == 3) && (stats[3] == 3)) {
      break;
    }
    Serial.print("  Sys "); Serial.print(stats[0]);
    Serial.print("  Gyr "); Serial.print(stats[1]);
    Serial.print("  Acc "); Serial.print(stats[2]);
    Serial.print("  Mag "); Serial.println(stats[3]);
    delay(500);
  }
}

void loop() {
  // put your main code here, to run repeatedly:
  char BTcommand ;
  
  while(!newData) { 
    // Waiting for Interrupt, can maybe do some stuff here
  }

  /* Push inputs into PID Controller, get outputs*/
  PID_P.SetTunings(consKpP, consKiP, consKdP);
  PID_R.SetTunings(consKpR, consKiR, consKdR);
  InputP = ypr[1] ;
  InputR = ypr[2] ;
  SetpointR = Roll_Setpoint ;
  SetpointP = Pitch_Setpoint ;
  PID_P.Compute();
  PID_R.Compute();
  /* End PID */

 /* Take PID output, Start Motor Update Sequence */
  rollOffset = u2deltaPWM_R * OutputR ;
  pitchOffset = u2deltaPWM_P * OutputP ;

  if (BT.available()) {
    BTcommand = (char)BT.read() ;
    BT.flush() ;
 /*
    Do something with BT command
    characters used: u, d, k, p, l, r, f, h, m, n, b, v, c, x
 */
    switch (BTcommand) {
      case 'm' : // Increase Kp for roll and pitch
        KpR = KpR + 0.1 ;
        KpP = KpP + 0.1 ;
        break ;
      case 'n' : // Decrease Kp for roll and pitch
        KpR = KpR - 0.1 ;
        KpP = KpP - 0.1 ;
        break ;
      case 'b' : // Increase Kd for roll and pitch
        KdR = KdR + 0.01 ;
        KdP = KdP + 0.01 ;
        break ;
      case 'v' : // Decrease Kd for roll and pitch
        KdR = KdR - 0.01 ;
        KdP = KdP - 0.01 ;
        break ;
      case 'c' : // Increase Ki for roll and pitch
        KiR = KiR + 0.001 ;
        KiP = KiP + 0.001 ;
        break ;
      case 'x' : // Decrease Ki for roll and pitch
        KiR = KiR - 0.001 ;
        KiP = KiP - 0.001 ;
        break ;
      case 'u' :
        basePWM=basePWM+0.5 ;
        break ;
      case 'd' :
        basePWM=basePWM-0.5;
        break ;
      case 'k' :
        motorControl.stopAll();
        break ;
      case 'p' :
        Pitch_Setpoint=Pitch_Setpoint+0.25;
        break;
      case 'l' :
        Pitch_Setpoint=Pitch_Setpoint-0.25;
        break;
      case 'r' :
        Roll_Setpoint=Roll_Setpoint-0.25;
        break;
      case 'f' :
        Roll_Setpoint=Roll_Setpoint+0.25;
        break;
      case 'h' :
        Roll_Setpoint = 0.0;
        Pitch_Setpoint = 0.0 ;
        break ;
    }
  }
    
  motorControl.setNS(basePWM, pitchOffset, yawOffset) ;
  motorControl.setEW(basePWM, rollOffset, yawOffset) ;
  /* End motor update sequence */
  
  /* Print for data/debugging*/
  Serial.println("");
  Serial.print("Yaw Actual     = ");Serial.print(ypr[0]);Serial.print("\n");
  Serial.print("Pitch Actual   = ");Serial.print(ypr[1]);Serial.print("\n");
  Serial.print("Roll Actual    = ");Serial.print(ypr[2]);Serial.print("\n");Serial.print("\n");
  Serial.print("Pitch Setpoint = ");Serial.print(Pitch_Setpoint);Serial.print("\n");
  Serial.print("Roll Setpoint  = ");Serial.print(Roll_Setpoint);Serial.print("\n");Serial.print("\n");
  Serial.println("    Roll     Pitch") ;
  Serial.print  ("Kp  ");
  Serial.print(KpR);Serial.print("   ");Serial.print(KpP);Serial.print("\n");
  Serial.print  ("Ki  ");
  Serial.print(KiR);Serial.print("   ");Serial.print(KiP);Serial.print("\n");
  Serial.print  ("Kd  ");
  Serial.print(KdR);Serial.print("   ");Serial.print(KdP);Serial.print("\n\n");
   
  newData = FALSE ;
}
