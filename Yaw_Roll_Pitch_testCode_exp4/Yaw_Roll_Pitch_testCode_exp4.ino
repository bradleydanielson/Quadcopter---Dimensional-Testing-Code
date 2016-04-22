/* 
  PID for Yaw, Roll, and Pitch Axis
  QC Project EWU 2016
  Written by Brad Danielson and Rudy Hulse
  Allows for bluetooth/serial control of testing PID parameters of motors on a physical testing rig

  Revision 0: created 
  Revision 1: Added custom "Motors" class
  Revision 2: Made PID parameters variables to be controlled remotely
  Revision 3: Prints to bluetooth device console
  Revision 4: Made kill command reset basePWM to minPWM, created a "Reset" option
  Revision 5: Updated new character commands, added Yaw 4/12
  Revision 6: Broke things up into functions 4/14
  Revision 7: Added rotateAxes() to get rid of yaw discontinuity 4/17
  
  future plans: 
    a)

  Instructions:
  1) Load sketch onto Teensy (properly comment out first or second line depending on using Bluetooth or USB serial)
  2) Connect Teensy with connected components to external power (3.7 V src)
  3) Connect all wiring on QC
  4) Once power is connected, LED on Teensy will turn on when IMU is calibrated (shake QC around to calibrate it, or wait 30 sec)
  5) Now it is just waiting for a lower case 'g' to be sent via bluetooth/serial to start motors
  6) PWM is at 0%, just use BT serial commands such as 'u' to change motor status (will print status at "displayPeriod" seconds)
  7) Initial Pitch and Roll Setpoints is at 0.0 degrees (hover), yaw is sampled at beginning at setpoint is initially sampled value. increase base PWM until QC hovers on it's own while connected to testing rig
  8) Change yaw, pitch, and roll setpoints to an offset from hover so that the angle of attack changes noticeably
  9) Command a hover mode by sending 'h' or 'H' and see how vehicle recovers

  Character Commands:
  PID > (lower case for down, Upper for up)
  
          R    P    Y
    Kp >  m    n    b
    Ki >  v    c    x
    Kd >  z    s    a

  Setpoints >
  P > Increase Pitch Setpoint by 0.25
  p > Decrease Pitch Setpoint by 0.25
  R > Increase Roll Setpoint by 0.25
  r > Decrease Roll Setpoint by 0.25
  Y > Increase Yaw Setpoint by 0.25
  y > Decrease Yaw Setpoint by 0.25
  h > Set P and R Setpoints to 0.0 degrees
  H > Set P, R, Y Setpoints to 0.0 degrees

  General Motor Control >
  g > start
  u > increase base PWM by 0.5
  d > decrease base PWM by 0.5
  k > kill all motors
  j > reset all parameters


*/

#include <PID_v1.h>
#include <i2c_t3.h>
#include <XYZ_BNO055.h>
#include <Motors.h>

/* comment out 
   Serial  > if using Bluetooth device
   Serial2 > if hardwired via USB using COM port
*/
//#define BT Serial
#define BT Serial2
/* */
#define TimeIntervalMilliSeconds 10  // Milliseconds 100 Hz
#define ms2us 1000 // us per ms 
#define u2deltaPWM_R 1
#define u2deltaPWM_P 1
#define TRUE 1 
#define FALSE 0
#define PID_MAX_VALUE 100
#define bitRes 65535
#define LEDPIN 13
#define displayPeriod 3 // seconds per debugging display print
#define deltaKP .01
#define deltaKI .0001
#define deltaKD .001
#define deltaBase 0.25
#define deltaSP 0.25

/* GLOBAL VARIABLES*/
double KiP = 0.001, KiR = 0.001, KpP = .1, KpR = .1, KdP = 0.01, KdR = 0.01, KiY = 0.0, KdY = 0.001, KpY = 0.1;
double InputP, OutputP ;
double InputR, OutputR ;
double InputY, OutputY ; 
float basePWM = 0 ;
float ypr[3] ;
float rollOffset, pitchOffset, yawOffset;
double Roll_Setpoint = 0.0, Pitch_Setpoint = 0.0, Yaw_Setpoint, Yaw_Setpoint_cons = 180.0;
int cnt = 0 ;
volatile int newData = FALSE ;
/* END GLOBAL VARIABLES */

/* Object Creation */
XYZ_BNO055 imu ;
PID PID_P(&InputP, &OutputP, &Pitch_Setpoint, KpP, KiP, KdP, DIRECT);
PID PID_R(&InputR, &OutputR, &Roll_Setpoint, KpR, KiR, KdR, DIRECT);
PID PID_Y(&InputY, &OutputY, &Yaw_Setpoint_cons, KpY, KiY, KdY, DIRECT);
Motors motorControl ;
IntervalTimer IMUUpdate ;
/* End Object Creation */

/* BEGIN INITIAL SETUP */
void setup() {
  /*Declarations*/
  char go ;
  pinMode(LEDPIN, OUTPUT);
  
  /* Setup The BNO055, Bluetooth, and I2C port */
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
  /* End IMU/BT setup */

  /* Setup The Motors */
  basePWM = 40.0 ;
  motorControl.initMotors() ;
  /* End Motor Setup */
  
  /* Setup The PID */
  //Pitch
  PID_P.SetMode(AUTOMATIC);
  PID_P.SetOutputLimits(-PID_MAX_VALUE, PID_MAX_VALUE);
  PID_P.SetSampleTime(TimeIntervalMilliSeconds);
  PID_P.SetTunings(KpP, KiP, KdP);

  //Roll
  PID_R.SetMode(AUTOMATIC);
  PID_R.SetOutputLimits(-PID_MAX_VALUE, PID_MAX_VALUE);
  PID_R.SetSampleTime(TimeIntervalMilliSeconds);
  PID_R.SetTunings(KpR, KiR, KdR);

  //Yaw
  PID_Y.SetMode(AUTOMATIC);
  PID_Y.SetOutputLimits(-PID_MAX_VALUE, PID_MAX_VALUE);
  PID_Y.SetSampleTime(TimeIntervalMilliSeconds);
  PID_Y.SetTunings(KpY, KiY, KdY);
  /* End PID Setup */  

  /* PWM Setup */
  analogWriteResolution(16) ;
  /* End PWM Setup */
  
  /* Get ready to start */
  BT.println("'g' to begin");
  while(true) {
    if(BT.available()) {
        go = (char)BT.read() ;
        if (go == 'g')
          break ;
    }  
    Serial.println("waiting for 'g'...");
    
    delay(1000);
  }

  imu.readYPR(ypr);
  Yaw_Setpoint = ypr[0];
  InputY = rotateAxes(Yaw_Setpoint, Yaw_Setpoint);
  

  /* Timer Interrupt Setup */
  // Interrupt is armed after this call
  IMUUpdate.begin(imuISR, TimeIntervalMilliSeconds * ms2us) ;
  /* End Interrupt Setup */
}
/* END INITIAL SETUP */

/* BEGIN MAIN LOOP */
void loop() { 
  if (newData == TRUE) ;
    /* If Interrupt has fired */
  {
    /* Read Pitch, Roll, and Yaw */
    imu.readYPR(ypr) ;
    
    /* Push inputs into PID Controllers, get outputs*/
    PID_P.SetTunings(KpP, KiP, KdP);
    PID_R.SetTunings(KpR, KiR, KdR);
    PID_Y.SetTunings(KpY, KiY, KdY);
    float Yaw_Actual = ypr[0] ;
    InputY = rotateAxes(Yaw_Setpoint, Yaw_Actual);
    InputP = ypr[1] ;
    InputR = ypr[2] ;
    PID_Y.Compute();
    PID_P.Compute();
    PID_R.Compute();
    /* End PID */
  
   /* Take PID output, Start Motor Update Sequence */
    yawOffset = -u2deltaPWM_R * OutputY ;
    rollOffset = u2deltaPWM_R * OutputR ;
    pitchOffset = u2deltaPWM_P * OutputP ;
  
    motorControl.setNS(basePWM, pitchOffset, yawOffset) ;
    motorControl.setEW(basePWM, rollOffset, yawOffset) ;
    /* End motor update sequence */
    
    newData = FALSE ;
    cnt++ ;
  }
  /* Get New Commands */
  if (BT.available()) {
    getBT();
  }
  
  if (cnt == displayPeriod*1000) {  
    /* Print for data/debugging*/
    printDebug();
  }
}
/* END MAIN LOOP */

/* FUNCTIONS */

//rotates the axes so that the setpoint is always 180 degrees and the actual has an even range on either side of the setpoint
float rotateAxes(float YawSP, float Yaw) {
  float diff, YawT ;
  diff = YawSP - 180.0 ;
  YawT = Yaw - diff;
  //fixes boundary problem as Yaw Actual crosses 0 degrees heading in the negative direction
  if (YawT < 0.0){YawT = YawT+360.0;}
  //fixes boundary problem as Yaw Actual crosses 359.9999 degrees heading in the positive direction
  else if(YawT > 359.9999){YawT = YawT-360;}
  return YawT;
}

void getBT( void ) {
  char BTcommand ;
  BTcommand = (char)BT.read() ;
    BT.flush() ;
    
    switch (BTcommand) {
    // Kp Roll
      case 'm' :
        KpR = KpR - deltaKP ;
        break ;
      case 'M' :
        KpR = KpR + deltaKP ;
        break ;
    // Kp Pitch
      case 'n' :
        KpP = KpP - deltaKP ;
        break ;
      case 'N' :
        KpP = KpP + deltaKP ;
        break;
    // Kp Yaw
      case 'b' :
        KpY = KpY - deltaKP ;
        break ;
      case 'B' :
        KpY = KpY + deltaKP ;
        break ;
    
    // Ki Roll
      case 'v' :
        KiR = KiR - deltaKI ;
        break ;
      case 'V' :
        KiR = KiR + deltaKI ;
        break ;
    // Ki Pitch
      case 'c' : 
        KiP = KiP - deltaKI ;
        break ;
      case 'C' :
        KiP = KiP + deltaKI;
        break ;
    // Ki Yaw
      case 'x' :
        KiY = KiY - deltaKI ;
        break ;
      case 'X' :
        KiY = KiY + deltaKI ;
        break ;

    // Kd Roll
      case 'z' :
        KdR = KdR - deltaKD ;
        break ;
      case 'Z' :
        KdR = KdR + deltaKD ;
        break ;
    // Kd Pitch
      case 's' :
        KdP = KdP - deltaKD ;
        break ;
      case 'S' :
        KdP = KdP + deltaKD ;
        break ;
    // Kd Yaw
      case 'a' :
        KdY = KdY - deltaKD ;
        break ;
      case 'A' :
        KdY = KdY + deltaKD ;
        break ;
        
    // Base PWM commands
      case 'u' :
        basePWM=basePWM+deltaBase ;
        break ;
      case 'd' :
        basePWM=basePWM-deltaBase;
        break ;
      case 'k' :
        motorControl.stopAll();
        basePWM = 40.0 ;
        while ( (char)BT.read() != 'g' ) {
          BT.flush() ;  
        }
        PID_P.ResetOutput();
        PID_R.ResetOutput();
        Yaw_Setpoint = ypr[0];
        InputY = rotateAxes(Yaw_Setpoint, InputY);
        PID_Y.ResetOutput();
        break ;
     // Setpoints
      case 'P' :
        Pitch_Setpoint = Pitch_Setpoint + deltaSP;
        break;
      case 'p' :
        Pitch_Setpoint = Pitch_Setpoint - deltaSP;
        break;
      case 'r' :
        Roll_Setpoint = Roll_Setpoint - deltaSP;
        break;
      case 'R' :
        Roll_Setpoint = Roll_Setpoint + deltaSP;
        break;
      case 'Y' :
        Yaw_Setpoint = Yaw_Setpoint + deltaSP ;
        if (Yaw_Setpoint < 0)
            Yaw_Setpoint = Yaw_Setpoint + 360.0 ;
        else if (Yaw_Setpoint > 360.0)
            Yaw_Setpoint = Yaw_Setpoint - 360.0 ;
        break ;
      case 'y' :
        Yaw_Setpoint = Yaw_Setpoint - deltaSP ;
        if (Yaw_Setpoint < 0)
            Yaw_Setpoint = Yaw_Setpoint + 360.0 ;
        else if (Yaw_Setpoint > 360.0)
            Yaw_Setpoint = Yaw_Setpoint - 360.0 ;
        break ;
      case 'h' :
        Roll_Setpoint = 0.0;
        Pitch_Setpoint = 0.0 ;
        break ;
      case 'H' :
        Roll_Setpoint = 0.0;
        Pitch_Setpoint = 0.0 ;
        Yaw_Setpoint = 0.0 ;
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
      case '1' :
        Yaw_Setpoint = 25 ;
        break ;
      case '2' :
        Yaw_Setpoint = 45 ;
        break ;
      case '3' :
        Yaw_Setpoint = 90 ;
        break ;
      case '4' :
        Yaw_Setpoint = 135 ;
        break ;
      case '5' :
        Yaw_Setpoint = 270 ;
        break ;
      case '6' :
        Yaw_Setpoint = 315 ;
        break ;
    } 
}
/* END MAIN LOOP */
void printDebug ( void ) {
  
      /* Print for data/debugging*/
    BT.println("------------------------------------------");
    BT.print("Base PWM = ");BT.print(basePWM);BT.print("%");
    BT.print("\n\n");
    BT.print("Yaw Actual     = ");BT.print(ypr[0],4);BT.print("  ");BT.print(InputY);BT.println();
    BT.print("Pitch Actual   = ");BT.print(ypr[1],4);BT.println();
    BT.print("Roll Actual    = ");BT.print(ypr[2],4);BT.println();BT.println();
    BT.print("Yaw Setpoint   = ");BT.print(Yaw_Setpoint,4);BT.println();
    BT.print("Pitch Setpoint = ");BT.print(Pitch_Setpoint,4);BT.println();
    BT.print("Roll Setpoint  = ");BT.print(Roll_Setpoint,4);BT.println() ;
    BT.println();
    BT.print("Error Signal\n");BT.print("Pitch =  ");BT.print(pitchOffset,4);BT.println();
    BT.print("Roll =  ");BT.print(rollOffset,4);BT.println();
    BT.print("Yaw  =  ");BT.print(yawOffset,4);BT.println();
    BT.println("    Roll     Pitch     Yaw") ; 
    BT.print  ("Kp  ");
    BT.print(KpR,4);BT.print("   ");BT.print(KpP,4);BT.print("   ");BT.print(KpY,4);BT.print("\n");
    BT.print  ("Ki  ");
    BT.print(KiR,4);BT.print("   ");BT.print(KiP,4);BT.print("   ");BT.print(KiY,4);BT.print("\n");
    BT.print  ("Kd  ");
    BT.print(KdR,4);BT.print("   ");BT.print(KdP,4);BT.print("   ");BT.print(KdY,4);BT.print("\n\n");
    cnt = 0;
}

/* IMU Calibration Function, runs for 30 seconds max, LED turns on when done */
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
/* End IMU Calibration Function */

/* IMU Read Interrupt Service Routine */
void imuISR ( void ) {
  newData = TRUE ;
}
/* End IMU ISR */
