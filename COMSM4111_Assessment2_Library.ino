#include "timer3.h"     // setup/isr for timer3 to calculate wheel speed.
#include "encoders.h"   // setup and isr to manage encoders.
#include "kinematics.h" // calculates x,y,theta from encoders.
#include "motor.h"      // handles power and direction for motors.
#include "pid.h"        // PID implementation.
#include "LineSensor.h" // handles all 3 line sensors as a single class.
#include "mapping.h"    // Used to store and read a metric byte map to EEPROM.
#include "utils.h"      // Used to generate random gaussian numbers.
#include "irproximity.h"// Used for the ir distance sensor.
#include "flood_fill.h"

//#include "imu.h"          // Advanced, work through labsheet if you wish to use.
//#include "magnetometer.h" // Advanced, work through labsheet if you wish to use.

#include <USBCore.h>    // To fix serial print behaviour bug.
u8 USB_SendSpace(u8 ep);
#define SERIAL_ACTIVE (USB_SendSpace(CDC_TX) >= 50)

/*****************************************************************************
    DEFINITIONS (global)
    Note, pins taken from the pin mapping for Romi available online.
*****************************************************************************/


#define M0_DIR          16  // Motor Pins.
#define M0_PWM          10
#define M1_DIR          15
#define M1_PWM          9

#define L_SENSE_L       A4  // Line sensor pins
#define L_SENSE_C       A3
#define L_SENSE_R       A2

#define BUZZER_PIN      6   // To make the annoying beeping

#define IR_PROX_PIN    A0   // IR Sensor

#define DEBUG_LED      13   // Using the orange LED for debugging

#define BUTTON_A       14   // Push button labelled A on board.
#define BUTTON_B       30   // Push button labelled B on board.

// Behaviour parameters
#define LINE_THRESHOLD        450.00
#define STRAIGHT_FWD_SPEED    0.1
#define LINE_FOLLOW_SPEED     4.0
#define IR_DETECTD_THRESHOLD  80   // a close reading in mm (danger)
#define IR_AVOIDED_THRESHOLD  140   // a distant reading in mm (safe)

// Speed controller for motors.
// Using same gains for left and right.
#define SPD_PGAIN     3.5
#define SPD_IGAIN     0.1
#define SPD_DGAIN     -1.5

// PID controller gains for heading feedback
#define H_PGAIN   1.8
#define H_IGAIN   0.0001
#define H_DGAIN   0.0

LineSensor  LineSensor( L_SENSE_L, L_SENSE_C, L_SENSE_R );  // Class to handle all 3 line sensors.
Motor       L_Motor( M0_PWM, M0_DIR);                       // To set left motor power.
Motor       R_Motor( M1_PWM, M1_DIR);                       // To set right motor power.
PID         L_PID( SPD_PGAIN, SPD_IGAIN, SPD_DGAIN );       // Speed control, left.
PID         R_PID( SPD_PGAIN, SPD_IGAIN, SPD_DGAIN );       // Speed control, right.
PID         H_PID( H_PGAIN, H_IGAIN, H_DGAIN );             // Position control, angle.
SharpIR     IRSensor0( IR_PROX_PIN );                       // Get distance to objects (incomplete class)
Kinematics  RomiPose;                                       // Not using ICC.
Mapper      Map;                                            // Default: 25x25 grid, 72mm resolution.
FloodFill ff;

unsigned long update_t;   // Used for timing/flow control for main loop()
unsigned long behaviour_t;// Use to track how long a behaviour has run.

// used by timer3.h to calculate left and right wheel speed.
volatile float l_speed_t3, r_speed_t3;

// Different states(behaviours) the robot
// can be in.
int STATE;
Coordinate curr;

#define STATE_CALIBRATE       0    // calibrates line sensor
#define STATE_INITIAL         1     // picks a random next state
#define STATE_FOLLOW_LINE     2     // Basic line following.
#define STATE_RANDOM_WALK     3     // Robot will make random turns 6 seconds
#define STATE_DRIVE_STRAIGHT  4     // Robot drives in a straight line 3 seconds
#define STATE_TURN_TO_ZERO    5     // Turns so the robot faces theta = 0
#define STATE_TURN_TO_PIOVER2 6     // Turns so the robot faces theta = PI/2 (90*)
#define STATE_AVOID_OBSTACLE  7

void initFloodFillState(){
  curr = {0, 0};
  ff.addToStack(Coordinate{0, 0});
}

void setup() {
  pinMode( BUZZER_PIN, OUTPUT );
  pinMode(DEBUG_LED, OUTPUT );
  pinMode( BUTTON_A, INPUT );
  digitalWrite( BUTTON_A, HIGH );
  pinMode( BUTTON_B, INPUT );
  digitalWrite( BUTTON_B, HIGH );
  
  setupEncoder0();
  setupEncoder1();
  setupTimer3();
  RomiPose.setPose( 0, 0, PI);
  
  initFloodFillState();

  Serial.begin(9600);
  delay(1000);
  
  //beep(); beep(); beep();
  if ( SERIAL_ACTIVE ) Serial.println("***RESET***");
}

void loop() {
  while (!ff.isEmpty()) {
    RomiPose.update( e0_count, e1_count );
    Coordinate tgt = ff.getCoordinate();
    if (!ff.visited(tgt)) {
      Serial.print("tgt: ");
      Serial.print(tgt.x);
      Serial.print(", ");
      Serial.println(tgt.y);
      RomiPose.update(e0_count, e1_count);
      moveToNextDestination(tgt);      
      curr = tgt;
      ff.addToVisited(tgt);
      Neighbours n = ff.getNeighbours(tgt);
      for (int i = 0; i < 4; i++) {
        if (ff.validateCoordinate(n.neighbours[i])) {
          ff.addToStack(n.neighbours[i]);
        }
      }
    }
  }
  delay(1);
}

void moveNSquares(int movement){
  for (int i = 0; i < movement; i++){
    moveForwards();
    RomiPose.update(e0_count, e1_count);
    delay(1000);
  }
}

void yMotionHandler(int diffOnY){
  if (diffOnY == 0) return;
  Coordinate rotationCoord;
  if (diffOnY < 0){
    rotationCoord = Coordinate{curr.x, curr.y - 1};
    //rotateTo(rotationCoord );
    while (!turnToTheta(0)) {
      RomiPose.update(e0_count, e1_count );
    }
  } else if (diffOnY > 0){
    rotationCoord = Coordinate{curr.x, curr.y + 1};
    while (!turnToTheta(PI)) {
      RomiPose.update(e0_count, e1_count );
    }
  } 
  stopMotors();
  delay(500);
  beep();
  delay(500);
  moveNSquares(abs(diffOnY));
}

void xMotionHandler(int diffOnX){
  if (diffOnX == 0) return;
  Coordinate rotationCoord;
  if (diffOnX < 0){
    rotationCoord = Coordinate{curr.x - 1, curr.y};
    rotateTo(rotationCoord );
  } else if (diffOnX > 0){
    rotationCoord = Coordinate{curr.x + 1, curr.y};
    rotateTo(rotationCoord );
  } 
  stopMotors();
  delay(500);
  beep();
  delay(500);
  moveNSquares(abs(diffOnX));
}

void moveToNextDestination(Coordinate tgt){
  int diffOnX = tgt.x - curr.x;
  int diffOnY = tgt.y - curr.y;

  xMotionHandler(diffOnX);
  yMotionHandler(diffOnY);  
}

void stopMotors() {
  L_Motor.setPower(0);
  R_Motor.setPower(0);
}

void moveForwards() {
  L_Motor.setPower(20);
  R_Motor.setPower(20);
  delay(1000);
  L_Motor.setPower(0);
  R_Motor.setPower(0);
}

void silentGoTo(Coordinate tgt)
{
  moveForwards();
  curr = tgt;
  RomiPose.update(e0_count, e1_count);
  delay(1000);
}

void rotateTo(Coordinate tgt)
{
  int diff_x = curr.x - tgt.x;
  int diff_y = curr.y - tgt.y;
  float err = 0.1;
  RomiPose.update(e0_count, e1_count);
  if (abs(diff_x) > 1 || abs(diff_y) > 1 || abs(diff_y) + abs(diff_x) > 1) {
    return;
    while(1){
      Serial.println("Finished Base");
    }
  }

  if (diff_x < 0) {
    Serial.println("Rotate to PI/2");
    if (RomiPose.theta > PI / 2 + err || RomiPose.theta < PI / 2 - err)
    {
      while (!turnToTheta(PI / 2)) {
        RomiPose.update(e0_count, e1_count );
      }
    }
    stopMotors();
    return;
  }
  else if (diff_x > 0) {
    Serial.println("Rotate to 3PI/2");
    if (RomiPose.theta > 3 * PI / 2 + err || RomiPose.theta < 3 * PI / 2 - err) {
      while (!turnToTheta(3 * PI / 2)) {
        RomiPose.update(e0_count, e1_count );
      }
    }
    stopMotors();
    return;
  }

  else if (diff_y < 0) {
    //Rotate to PI
    Serial.println("Rotate to PI");
    if (RomiPose.theta > PI  + err || RomiPose.theta < PI - err)
    {
      while (!turnToTheta(PI)) {
        RomiPose.update(e0_count, e1_count );
      }
    }
    stopMotors();
    return;

  }
  else if (diff_y > 0) {
    //Rotate to 0
    Serial.println("Rotate to 0");
    if (RomiPose.theta >  err && RomiPose.theta <  2 * PI - err) {
      while (!turnToTheta(0)) {
        RomiPose.update(e0_count, e1_count );
      }
    }
    stopMotors();
    return;
  }

}
void goTo(Coordinate tgt) {
  moveForwards();
  curr = tgt;
  RomiPose.update(e0_count, e1_count);
  delay(1000);
}


void decideStartUpFromButtons() {
  int mode = -1;
  do {

    if ( SERIAL_ACTIVE ) Serial.println("Waiting for button a (print map) or b (erase map)");

    int btn_a = digitalRead( BUTTON_A );
    int btn_b = digitalRead( BUTTON_B );

    // Decide if we are going to print
    // or erase the map.
    if ( btn_a == LOW ) {
      mode = 0;
    } else if ( btn_b == LOW ) {
      mode = 1;
    }

  } while ( mode < 0 );

  // Acknowledge button press.
  beep();

  if ( mode == 0 ) {  // Print map

    // Because 1 will always be true, you Romi
    // will no be stuck in this loop forever.
    while ( 1 ) {
      Map.printMap();
      delay(2000);
    }

  }

  if ( SERIAL_ACTIVE ) Serial.println("Erasing Map, activating Romi");

  Map.resetMap();

}

void beep() {
  analogWrite(6, 30);
  delay(50);
  analogWrite(6, 0);
  delay(50);
}

void reportUnknownState() {
  if ( SERIAL_ACTIVE ) {
    Serial.print("Unknown state: ");
    Serial.println( STATE );
  }
}

void changeState( int which ) {

  if ( which == STATE ) return;

  L_Motor.setPower( 0 );
  R_Motor.setPower( 0 );
  behaviour_t = millis();
  update_t = millis();
  STATE = which;

  L_PID.reset();
  R_PID.reset();
  H_PID.reset();

  return;
}

void calibrateSensors() {
  L_Motor.setPower( 0 );
  R_Motor.setPower( 0 );
  LineSensor.calibrate();
  changeState( STATE_INITIAL );
}

bool turnToTheta(float demand_angle) {
  float diff = atan2( sin( ( demand_angle - RomiPose.theta) ), cos( (demand_angle - RomiPose.theta) ) );
  if ( abs( diff ) < 0.03 ) {
    return true;
  } else {
    float bearing = H_PID.update( 0, diff );
    float l_pwr = L_PID.update( (0 - bearing), l_speed_t3 );
    float r_pwr = R_PID.update( (0 + bearing), r_speed_t3 );
    
    L_Motor.setPower(l_pwr);
    R_Motor.setPower(r_pwr);
    return false;
  } 
}
