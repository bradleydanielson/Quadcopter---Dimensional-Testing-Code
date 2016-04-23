/* 
  Position Controller
  QC/CP Project EWU 2016
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
  Revision 8: Added rotateVector() to calculate pitch and roll contributions to position controller in 2D 4/21
  
  future plans: 
    a)

  Instructions:
  1) Load sketch onto Teensy (properly comment out first or second line depending on using Bluetooth or USB serial)
  2) Connect Teensy with connected components to external power (3.7 V src)
  3) Connect all wiring on QC
  4) Once power is connected, LED on Teensy will turn on when IMU is calibrated (shake QC around to calibrate it, or wait 30 sec)
  5) Now it is just waiting for a lower case 'g' to be sent via bluetooth/serial to start motors
  6) PWM is at 40.0% (0% Thrust), just use BT serial commands such as 'u' to change motor status (will print status at "displayPeriod" seconds)
  7) Initial State is hover in place
  8) Change Where the vehicle will go with by changing radius and angle of heading vector with 'r' and 't'
  9) Command a hover mode by sending 'h' or 'H' and see how vehicle recovers
  
                                        0
                                        N (actual)
                                        
                                270 W       E 90
                                        
                                        S
                                       180

  Character Commands:
  PID > (lower case for down, Upper for up)
  
          R    P    Y    X    Y
    Kp >  m    n    b    o    y
    Ki >  v    c    x    e    r
    Kd >  z    s    a    q    w

  Setpoints (QC assumes it is at (0,0))
    x = R*cos(Theta), y = R*sin(Theta) >
    Radius r, R (--,++)
    Theta  t, T (--,++)  (0 degrees is relative to N)

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
#include <math.h>

/*
   * COMMENT OUT
   * Serial  > if using Bluetooth device
   * Serial2 > if hardwired via USB using COM port
*/
#define BT Serial
//#define BT Serial2
/* */
#define TimeIntervalMilliSeconds 10  // IMU Update Period Milliseconds 100 Hz
#define TimeInterval10HzTask 100 // GPS UpdatePeriod Milliseconds 10 Hz
#define ms2us 1000 // us per ms 
#define u2deltaPWM_R 1
#define u2deltaPWM_P 1
#define u2deltaPWM_Y -1 
#define TRUE 1 
#define FALSE 0
#define PID_MAX_VALUE 100
#define PID_MAX_ANGLE 10 
#define bitRes 65535 // PWM Bit Resolution
#define LEDPIN 13
#define displayPeriod 3 // seconds per debugging display print
#define deltaKP .01
#define deltaKI .0001
#define deltaKD .001
#define deltaBase 0.25
#define deltaSP 0.5
#define deg2rad 0.0174533

/* GLOBAL VARIABLES*/
double KiP = 0.001, KiR = 0.001, KpP = .1, KpR = .1, KdP = 0.01, KdR = 0.01, KiY = 0.0, KdY = 0.001, KpY = 0.1;
double KpN = 1.0, KiN = 0.00, KdN = 0.001, KpE = 1.0, KiE = 0.00, KdE = 0.001;
double OutputE, OutputN ;
double InputE_cons = 0.0, InputN_cons = 0.0 ; /* Zero (constants) due to rotateVector() algorithm */
double InputP = 0.0, OutputP ;
double InputR = 0.0, OutputR ;
double InputY, OutputY ; 
double R = 0.0, Theta = 0.0 ; /* Initial Conditions, R is how far you want to travel, Theta is direction */
float basePWM = 0 ;
float ypr[3] ;
float rollOffset, pitchOffset, yawOffset;
double Roll_Setpoint = 0.0, Pitch_Setpoint = 0.0, Yaw_Setpoint, Yaw_Setpoint_cons = 180.0;
double N_Setpoint = 0.0, E_Setpoint = 0.0, QCx = 0.0, QCy = 0.0 ;
int cnt = 0 ;
volatile int newDataIMU = FALSE, newDataGPS = FALSE ; /* Volatile Interrupt Variables */
/* END GLOBAL VARIABLES */

/* Object Creation */
XYZ_BNO055 imu ;
PID PID_N(&InputN_cons, &OutputN, &N_Setpoint, KpN, KiN, KdN, DIRECT) ;
PID PID_E(&InputE_cons, &OutputE, &E_Setpoint, KpE, KiE, KdE, DIRECT) ;
PID PID_P(&InputP, &OutputP, &Pitch_Setpoint, KpP, KiP, KdP, DIRECT);
PID PID_R(&InputR, &OutputR, &Roll_Setpoint, KpR, KiR, KdR, DIRECT);
PID PID_Y(&InputY, &OutputY, &Yaw_Setpoint_cons, KpY, KiY, KdY, DIRECT);
Motors motorControl ;

IntervalTimer IMUUpdate ;
IntervalTimer TenHzTask ;
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
  /* PID_MAX_ANGLE is 10 degrees */
  //N (NORTH/SOUTH AXIS) N = -S, E X N = Z
  PID_N.SetMode(AUTOMATIC);
  PID_N.SetOutputLimits(-PID_MAX_ANGLE, PID_MAX_ANGLE);
  PID_N.SetSampleTime(TimeInterval10HzTask);
  PID_N.SetTunings(KpN, KiN, KdN);

  //E (EAST/WEST AXIS) E = -W
  PID_E.SetMode(AUTOMATIC);
  PID_E.SetOutputLimits(-PID_MAX_ANGLE, PID_MAX_ANGLE);
  PID_E.SetSampleTime(TimeInterval10HzTask);
  PID_E.SetTunings(KpE, KiE, KdE);
  
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
  
  /* Configure Initial Yaw Command */
  imu.readYPR(ypr); 
  Yaw_Setpoint = ypr[0]; // Yaw Setpoint is where you want to point
  Theta = Yaw_Setpoint;  // Theta is direction you want to travel
  InputY = rotateAxes(Yaw_Setpoint, Yaw_Setpoint);
  
  /* Timer Interrupts Setup */
  // Interrupt is armed after this call
  IMUUpdate.begin(imuISR, TimeIntervalMilliSeconds * ms2us) ;
  TenHzTask.begin(TenHzISR, TimeInterval10HzTask * ms2us) ;
  /* End Interrupt Setup */
}
/* END INITIAL SETUP */

/* BEGIN MAIN LOOP */
void loop() { 
  
  /* 10 HERTZ TASK, SAMPLE GPS :: GET (X, Y) , DO PID_XY */
  if (newDataGPS == TRUE) {
    /* If GPS Position Interrupt has fired (10 Hz) */   
    PID_N.SetTunings(KpN, KiN, KdN) ;
    PID_E.SetTunings(KpE, KiE, KdE) ;
    imu.readYPR(ypr) ; // Sample Yaw
    rotateVector( R, Theta, ypr[0], QCx, QCy) ;
    PID_N.Compute() ;
    PID_E.Compute() ;
    Roll_Setpoint = OutputE ;
    Pitch_Setpoint = OutputN ;
    newDataGPS = FALSE ;
  }
  /* End Position Update */

  /* 100 HERTZ TASK, GET YPR, DO PID_YPR */
  if (newDataIMU == TRUE)
    /* If IMU Interrupt has fired (100 Hz) */
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
    yawOffset = u2deltaPWM_Y * OutputY ;
    rollOffset = u2deltaPWM_R * OutputR ;
    pitchOffset = u2deltaPWM_P * OutputP ;
  
    motorControl.setNS(basePWM, pitchOffset, yawOffset) ;
    motorControl.setEW(basePWM, rollOffset, yawOffset) ;
    /* End motor update sequence */
    
    newDataIMU = FALSE ;
    cnt++ ;
  }
  /* Get New Commands */
  if (BT.available()) {
    getBT();
  }
  
  if (cnt == displayPeriod*100) {  
    /* Print for data/debugging*/
    printDebug();
  }
}
/* END MAIN LOOP */

/* FUNCTIONS */

/*  ROTATE VECTOR FUNCTION
 *  > Fixes roll/pitch setpoint problem, generates Error signal fed into position PID
 *  > After this function runs, position measured (N_input, E_input) must be 0.0 (PID INPUT)
 *  > Alters Global Variables E_Setpoint, N_Setpoint (input into position PID) 
*/
void rotateVector( double R, double Theta, double yaw, double QCx , double QCy ) {
  double TARGETx, TARGETy, ERRORx, ERRORy, ERRORx_T, ERRORy_T ;
  // Defines Target location based off of polar coordinates in the 2d plane (x,y)
  TARGETx = R*cos(deg2rad*Theta) ;
  TARGETy = R*sin(deg2rad*Theta) ;
  // Finds error vector
  ERRORx = TARGETx - QCx ;
  ERRORy = TARGETy - QCy ;
  // Rotates Error Vector
  ERRORy_T = ERRORy*cos(deg2rad*yaw) - ERRORx*sin(deg2rad*yaw) ;
  ERRORx_T = ERRORy*sin(deg2rad*yaw) + ERRORx*cos(deg2rad*yaw) ;
  // Transformed error vectors components are returned and assigned to E, N Setpoints
  E_Setpoint = -ERRORy_T ; /* Negative to compensate for 180 degree error in our initial logic */
  N_Setpoint = -ERRORx_T ;
}
/* END ROTATE VECTOR FUNCTION */

/* ROTATE AXES FUNCTION 
 *  > Fixes discontinuity at 0.0 <-> 360.0 degree boundary
 *  > After this function runs discontinuity always exists at 180.0 degrees from yaw setpoint
 *  > Yaw Setpoint in PID input (InputY) should be constant 180.0 degrees
*/
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
/* END ROTATE AXES FUNCTION */

/* GET BLUETOOTH COMMANDS FUNCTION */
void getBT( void ) {
  char BTcommand ;
  BTcommand = (char)BT.read() ;
    BT.flush() ;
    // Begin Ridiculously Long Switch Statement
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
     /* Ones that are commented out do not pertain to this test */
//      case 'P' :
//        Pitch_Setpoint = Pitch_Setpoint + deltaSP;
//        break;
//      case 'p' :
//        Pitch_Setpoint = Pitch_Setpoint - deltaSP;
//        break;
//      case 'r' :
//        Roll_Setpoint = Roll_Setpoint - deltaSP;
//        break;
//      case 'R' :
//        Roll_Setpoint = Roll_Setpoint + deltaSP;
//        break;
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
//      case 'h' :
//        Roll_Setpoint = 0.0;
//        Pitch_Setpoint = 0.0 ;
//        break ;
//      case 'H' :
//        Roll_Setpoint = 0.0;
//        Pitch_Setpoint = 0.0 ;
//        Yaw_Setpoint = 0.0 ;
//        break ;
//      case 'j' :
//        Roll_Setpoint = 0.0;
//        Pitch_Setpoint = 0.0 ;
//        KpP = 0.25 ;
//        KpR = 0.25 ;
//        KiP = 0.0 ;
//        KiR = 0.0 ;
//        KdP = 0.0 ;
//        KdR = 0.0 ;
//        break ;
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
      /* Change Direction/Magnitude of Travel Destination */
      case 'r' :
        R = R - 0.25 ;
        if (R < 0.0){R = 0;}
        break;
      case 'R' :
        R = R + 0.25 ;
        break ;
      case 't' :
        Theta = Theta - 1.0 ;
        if (Theta < 0.0)
          Theta = Theta + 360.0 ;
        break ;
      case 'T' :
        Theta = Theta + 1.0 ;
        if (Theta > 360.0)
          Theta = Theta - 360.0 ;
        break ;
    } 
}
/* END GET COMMANDS FUNCTION */

/* Print Values for Debugging */
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
    BT.print("Yaw  =  ");BT.print(yawOffset,4);BT.println();BT.println();
    BT.print("Distance to commanded position = ");BT.print(R);BT.print("\n");
    BT.print("Angle to commanded position    = ");BT.print(Theta);BT.print("\n\n");
    BT.println("                       N     E ");
    BT.print(  "Commanded Position = ");BT.print(R*cos(deg2rad*Theta));BT.print("  ");BT.print(R*sin(deg2rad*Theta));BT.print("\n\n");
    BT.println("    Roll     Pitch     Yaw") ; 
    BT.print  ("Kp  ");
    BT.print(KpR,4);BT.print("   ");BT.print(KpP,4);BT.print("   ");BT.print(KpY,4);BT.print("\n");
    BT.print  ("Ki  ");
    BT.print(KiR,4);BT.print("   ");BT.print(KiP,4);BT.print("   ");BT.print(KiY,4);BT.print("\n");
    BT.print  ("Kd  ");
    BT.print(KdR,4);BT.print("   ");BT.print(KdP,4);BT.print("   ");BT.print(KdY,4);BT.print("\n\n");
    cnt = 0;
}
/* End print function */

/* IMU Calibration Function, runs for 120 seconds max, LED turns on when done */
void calibrateIMU() {
  Serial.println("Cal: No=0, full=3");

  uint8_t stats[4];
  for (int i = 0; i < 240; i++) {
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
  newDataIMU = TRUE ;
}
/* End IMU ISR */

/* GPS Read Interrupt Service Routine */
void TenHzISR ( void ) {
  newDataGPS = TRUE ;
}
/* End GPS ISR */

