/* 
  PID for Roll and Pitch Axis
  QC Project EWU 2016
  Allows for bluetooth control of testing PID parameters of motors

  Revision 0: created 
  Revision 1: Added "Motors" class
  Revision 2: Made PID parameters variables to be controlled remotely
  Revision 3: Prints to bluetooth device console
  Revision 4: Made kill command reset basePWM to minPWM, created a "Reset" option

  Instructions:
  1) Load sketch onto Teensy
  2) Connect Teensy with connected components to external power (3.7 V src)
  3) Connect all wiring on QC
  4) Once power is connected, LED on Teensy will turn on when IMU is calibrated (shake QC around to calibrate it)
  5) Now it is just waiting for a lower case 'g' to be sent via bluetooth to start motors
  6) PWM is at 0%, just use BT serial commands such as 'u' to change motor status
  7) Initial Pitch and Roll Setpoints is at 0.0 degrees (hover), increase base PWM until QC hovers on it's own while connected to testing rig
  8) Change pitch or roll setpoint to an offset from hover so that the angle of attack changes noticeably
  9) Command a hover mode by sending 'h' and see how vehicle recovers

  Serial Commands:
  PID >
  m > increase Kp by 0.1
  n > decrease Kp by 0.1
  b > increase Kd by 0.01
  v > decrease Kd by 0.01
  c > increase Ki by 0.001
  x > decrease Ki by 0.001

  Setpoints >
  p > Increase Pitch Setpoint by 0.25
  l > Decrease Pitch Setpoint by 0.25
  r > Increase Roll Setpoint by 0.25
  f > Decrease Roll Setpoint by 0.25
  h > Set P and R Setpoints to 0.0 degrees

  General Motor Control >
  u > increase base PWM by 0.5
  d > decrease base PWM by 0.5
  k > kill all motors
  j > reset all parameters


*/

#include <PID_v1.h>
#include <i2c_t3.h>
#include <XYZ_BNO055.h>
#include <Motors.h>

//#define BT Serial
#define BT Serial2
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
int cnt = 0 ;
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
  delay(1500) ;
  BT.begin(115200) ;
  Serial.begin(115200) ;
  delay(150) ;
  BT.println("BNO055 TEST");
  while (!imu.setup(BNO055_ADDRESS_B))
  {
    BT.println("No BNO055 found");
    delay(100);
  }
  BT.println("BNO055 found") ;
  delay(1000) ;
  imu.setMode(XYZ_BNO055::NDOF) ;
  calibrateIMU();
  BT.println("IMU Calibration Success");
  delay(1000);
  digitalWrite(LEDPIN,HIGH);

  /* Setup Bluetooth */
  //BT.begin(115200) ;

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
  // Take this out of loop
  //imu.readYPR(ypr) ;
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
  
//  while(!newData) { 
//    // Waiting for Interrupt, can maybe do some stuff here
//  }

  if (newData == TRUE) ;
    {
    /* Read Pitch, Roll, and Yaw */
    imu.readYPR(ypr) ;
    
    /* Push inputs into PID Controller, get outputs*/
    PID_P.SetTunings(KpP, KiP, KdP);
    PID_R.SetTunings(KpR, KiR, KdR);
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
  
    motorControl.setNS(basePWM, pitchOffset, yawOffset) ;
    motorControl.setEW(basePWM, rollOffset, yawOffset) ;
    /* End motor update sequence */
    
    newData = FALSE ;
    cnt++;
  }

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
        basePWM = 40.0 ;
        while ( (char)BT.read() != 'g' ) {
          BT.flush() ;  
        }
//        PID_P.SetOutputLimits(0.0,0.0) ;
//        PID_R.SetOutputLimits(0.0,0.0) ;
//        OutputP = 0.0 ;
//        OutputR = 0.0 ;
//        PID_P.Compute();
//        PID_R.Compute();
//        PID_P.SetOutputLimits(-PID_MAX_VALUE,PID_MAX_VALUE) ;
//        PID_R.SetOutputLimits(-PID_MAX_VALUE,PID_MAX_VALUE) ;
        PID_P.ResetOutput();
        PID_R.ResetOutput();
        BT.print("\n shit is reset\n") ;
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
      case 'j' :
        Roll_Setpoint = 0.0;
        Pitch_Setpoint = 0.0 ;
        KpP = 0.25 ;
        KpR = 0.25 ;
        KiP = 0.0 ;
        KiR = 0.0 ;
        KdP = 0.0 ;
        KdR = 0.0 ;
        break ;
    }
  }
  

  
  if (cnt == 10000) {
  
    /* Print for data/debugging*/
    BT.println("");
    BT.print("Yaw Actual     = ");BT.print(ypr[0]);BT.print("\n");
    BT.print("Pitch Actual   = ");BT.print(ypr[1]);BT.print("\n");
    BT.print("Roll Actual    = ");BT.print(ypr[2]);BT.print("\n");BT.print("\n");
    BT.print("Pitch Setpoint = ");BT.print(Pitch_Setpoint);BT.print("\n");
    BT.print("Roll Setpoint  = ");BT.print(Roll_Setpoint);BT.print("\n");BT.print("\n");
    BT.print("Error Signal\n");BT.print("Pitch =  ");BT.print(pitchOffset);BT.print("\n");
    BT.print("Roll =  ");BT.print(rollOffset);BT.print("\n\n");
    BT.println("    Roll     Pitch") ;
    BT.print  ("Kp  ");
    BT.print(KpR);BT.print("   ");BT.print(KpP);BT.print("\n");
    BT.print  ("Ki  ");
    BT.print(KiR);BT.print("   ");BT.print(KiP);BT.print("\n");
    BT.print  ("Kd  ");
    BT.print(KdR);BT.print("   ");BT.print(KdP);BT.print("\n\n");
    cnt = 0;
  }
   
  
}
