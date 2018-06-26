//==========================================================================================
//
//  Program for controlling a mePed Robot using an IR Remote
//
//  The mePed is an open source quadruped robot designed by Scott Pierce of
//  Spierce Technologies (www.meped.io & www.spiercetech.com)
//
//  This program is based on code written by Alexey Butov (www.alexeybutov.wix.com/roboset)
//
//==========================================================================================

#include <IRremote.h> // include IR Remote library
#include <Servo.h>    // include servo library

//===== Globals ============================================================================

// Define USRF pins and variables
#define trigPin A3
#define echoPin A2
#define INCH 0
#define CM 1

// Speed constants
#define SPEED_MAX  10 // delay 5ms between movements
#define SPEED_MIN  1  // delay 50ms between movements
#define SPEED_STEP 1  // how quickly speed can be changed

// Height constants
#define HEIGHT_MAX    10
#define HEIGHT_MIN    0
#define HEIGHT_CENTER 5

// Initial values
#define INIT_SPEED  SPEED_MAX
#define INIT_HEIGHT HEIGHT_CENTER

// calibration
int da =  -12,  // Left Front Pivot
    db =   10,  // Left Back Pivot
    dc =  -18,  // Right Back Pivot
    dd =   12;  // Right Front Pivot

// servo initial positions + calibration
int servo_flp_center = 90 + da,
    servo_blp_center = 90 + db,
    servo_brp_center = 90 + dc,
    servo_frp_center = 90 + dd;
    
int a90, a120, a150, a180;  // Front Left Pivot
int b0, b30, b60, b90;      // Back Left Pivot
int c90, c120, c150, c180;  // Back Right Pivot
int d0, d30, d60, d90;      // Front Right Pivot

// current speed & height settings
int speed  = INIT_SPEED;  // Speed of walking motion, larger the number, the slower the speed
int height = INIT_HEIGHT; // How high the robot is standing

// Define 8 Servos
Servo servoFLPivot; // Front Left Pivot Servo
Servo servoFLLift; // Front Left Lift Servo
Servo servoBLPivot; // Back Left Pivot Servo
Servo servoBLLift; // Back Left Lift Servo
Servo servoBRPivot; // Back Right Pivot Servo
Servo servoBRLift; // Back Right Lift Servo
Servo servoFRPivot; // Front Right Pivot Servo
Servo servoFRLift; // Front Right Lift Servo

// Servo physical limits (change to suit application)
#define SERVO_LIFT_MAX    130 // max angle of leg lift allowed (shortest height)
#define SERVO_LIFT_MIN    50  // min angle of leg lift allowed (tallest height)
#define SERVO_LIFT_RANGE  (SERVO_LIFT_MAX - SERVO_LIFT_MIN)
#define SERVO_LIFT_BUFFER (int) (SERVO_LIFT_RANGE * 0.125) // angle buffer allows up/down movement

// calibrate lift servo positions
int servo_lift_up     = SERVO_LIFT_MIN;
int servo_lift_center = (SERVO_LIFT_MAX + SERVO_LIFT_MIN) / 2;
int servo_lift_down   = SERVO_LIFT_MAX;

// Set up IR Sensor
int irReceiverPin = 12;           // Use pin D12 for IR Sensor
IRrecv irReceiver(irReceiverPin); // create a new instance of the IR Receiver
decode_results irResponse;        // decoded IR signal

// Define remote control keys
enum RemoteKey {NONE,REPEATED,LEFT,RIGHT,UP,DOWN,OK,N0,N1,N2,N3,N4,N5,N6,N7,N8,N9,STAR,POUND};

//==========================================================================================

//===== Setup ==============================================================================

void setup()
{
  // Enable serial comms for debugging
  Serial.begin(9600);

  // Attach servos to Arduino Pins
  servoFLPivot.attach(2);
  servoFLLift.attach(3);
  servoBLPivot.attach(4);
  servoBLLift.attach(5);
  servoBRPivot.attach(6);
  servoBRLift.attach(7);
  servoFRPivot.attach(8);
  servoFRLift.attach(9);

  // Set USRF pins
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  // Enable the IR receiver
  irReceiver.enableIRIn();

  // initialize servo positions
  recalculate_servo_centers();

  // Center all servos
  center_servos();
  
}//setup

//==========================================================================================

//== Loop ==================================================================================

void loop()
{
  RemoteKey thisKey = NONE;
  RemoteKey lastKey = NONE;

  while (1 == 1)    // Loop forever
  {
    thisKey = get_keypress();

    // Have we received a new command?
    

    if (thisKey == NONE)
    {
      // skip rest of the loop
      continue;
    } 
    else if (thisKey == REPEATED)
    {
      thisKey = lastKey;
    }
    
    switch (thisKey)
    {
        case UP:
          forward();
          break;

        case DOWN:
          back();
          break;

        case RIGHT:
          turn_right();
          break;

        case LEFT:
          turn_left();
          break;

        case OK:
          // unassigned
          break;

        case N1:
          bow();
          break;

        case N2:
          wave();
          break;

        case N3:
          // increase speed
          change_speed(SPEED_STEP);
          break;

        case N4:
          // unassigned
          break;

        case N5:
          // unassigned
          break;

        case N6:
          // decrease speed
          change_speed(-SPEED_STEP);
          break;

        case N7:
          change_height(-1);
          break;

        case N8:
          dance();
          break;

        case N9:
          change_height(1);
          break;

        case N0:
          center_servos();
          break;

        case STAR:
          trim_left();
          break;

        case POUND:
          trim_right();
          break;

        default:
          break;
      }

      // store this command in case we need to repeat
      lastKey = thisKey;

      delay(50);  // Pause for 50ms before executing next movement

  }//while

}//loop

void dance()
{
  center_servos();
  delay(100);
  lean_left();
  delay(300);
  lean_right();
  delay(300);
  lean_left();
  delay(300);
  lean_right();
  delay(300);
  lean_left();
  delay(300);
  lean_right();
  delay(300);
  lean_left();
  delay(300);
  lean_right();
  delay(800);
  center_servos();
  delay(300);
  bow();
  center_servos();
}

//== IR Control Decoder ====================================================================

RemoteKey get_keypress()
{
  if (irReceiver.decode(&irResponse)) // If we have received an IR signal
  {
    // Use this debug code to add support for other IR remotes
    Serial.println("IR code: 0x" + String(irResponse.value, HEX));

    // clear for next input
    irReceiver.resume();

    switch (irResponse.value)
    {
      // enum RemoteKey {NONE,REPEATED,LEFT,RIGHT,UP,DOWN,OK,N0,N1,N2,N3,N4,N5,N6,N7,N8,N9,STAR,HASH,FF,RR,MENU,SETUP};

      // KEYES remote (came w/ bot)
      case 0xFF629D:
        return UP;
      case 0xFFA857:
        return DOWN;
      case 0xFFC23D:
        return RIGHT;
      case 0xFF22DD:
        return LEFT;
      case 0xFF02FD:
        return OK;
      case 0xFF6897:
        return N1;
      case 0xFF9867:
        return N2;
      case 0xFFB04F:
        return N3;
      case 0xFF30CF:
        return N4;
      case 0xFF18E7:
        return N5;
      case 0xFF7A85:
        return N6;
      case 0xFF10EF:
        return N7;
      case 0xFF38C7:
        return N8;
      case 0xFF5AA5:
        return N9;
      case 0xFF4AB5:
        return N0;
      case 0xFF42BD:
        return STAR;
      case 0xFF52AD:
        return POUND;
      case 0xFFFFFFFF: // repeat last key
        return REPEATED;
      default:
        return NONE;
    } // switch
    
  } 
  else
  {
    return NONE;
  }

}

//== Wave ==================================================================================

void wave()
{
  /*
  servoFLPivot - Front Left Pivot Servo
  servoFLLift - Front Left Lift Servo
  servoBLPivot - Back Left Pivot Servo
  servoBLLift - Back Left Lift Servo
  servoBRPivot - Back Right Pivot Servo
  servoBRLift - Back Right Lift Servo
  servoFRPivot - Front Right Pivot Servo
  servoFRLift - Front Right Lift Servo
  */

  center_servos();
  // tilt back
  servoBLLift.write(45);
  servoBRLift.write(45);
  delay(200);
  // lift leg
  servoFRLift.write(0);
  delay(200);
  // wave back and forth
  servoFRPivot.write(180);
  delay(200);
  servoFRPivot.write(30);
  delay(300);
  servoFRPivot.write(180);
  delay(300);
  servoFRPivot.write(30);
  delay(300);
  // back to center
  servoFRPivot.write(servo_frp_center);
  delay(300);
  // foot down
  servoFRLift.write(servo_lift_center);
  center_servos();

}

//== Bow ===================================================================================

void bow()
{
  center_servos();
  delay(200);
  servoFLLift.write(servo_lift_up);
  servoFRLift.write(servo_lift_up);
  delay(700);
  servoFLLift.write(servo_lift_center);
  servoFRLift.write(servo_lift_center);
  delay(700);
}

//== Lean_Left =============================================================================

void lean_left()
{
  servoFLLift.write(servo_lift_up);
  servoBLLift.write(servo_lift_up);
  servoBRLift.write(servo_lift_down);
  servoFRLift.write(servo_lift_down);
}

//== Lean_Right ============================================================================

void lean_right()
{
  servoFLLift.write(servo_lift_down);
  servoBLLift.write(servo_lift_down);
  servoBRLift.write(servo_lift_up);
  servoFRLift.write(servo_lift_up);
}

//== Trim Left =============================================================================

void trim_left()
{
  da--; // Left Front Pivot
  db--; // Left Back Pivot
  dc--; // Right Back Pivot
  dd--; // Right Front Pivot

  recalculate_servo_centers();
  center_servos();
}

//== Lean_Right ============================================================================

void trim_right()
{
  da++; // Left Front Pivot
  db++; // Left Back Pivot
  dc++; // Right Back Pivot
  dd++; // Right Front Pivot

  recalculate_servo_centers();
  center_servos();
}

//== Forward ===============================================================================

void forward()
{
  // calculation of points

  // Left Front Pivot
  a90 = (90 + da),
  a120 = (120 + da),
  a150 = (150 + da),
  a180 = (180 + da);

  // Left Back Pivot
  b0 = (0 + db),
  b30 = (30 + db),
  b60 = (60 + db),
  b90 = (90 + db);

  // Right Back Pivot
  c90 = (90 + dc),
  c120 = (120 + dc),
  c150 = (150 + dc),
  c180 = (180 + dc);

  // Right Front Pivot
  d0 = (0 + dd),
  d30 = (30 + dd),
  d60 = (60 + dd),
  d90 = (90 + dd);

  // set servo positions and speeds needed to walk forward one step
  // (LFP,  LBP, RBP,  RFP, LFL, LBL, RBL, RFL, S1, S2, S3, S4)
  srv(a180, b0 , c120, d60, 42,  33,  33,  42,  1,  3,  1,  1);
  srv( a90, b30, c90,  d30, 6,   33,  33,  42,  3,  1,  1,  1);
  srv( a90, b30, c90,  d30, 42,  33,  33,  42,  3,  1,  1,  1);
  srv(a120, b60, c180, d0,  42,  33,  6,   42,  1,  1,  3,  1);
  srv(a120, b60, c180, d0,  42,  33,  33,  42,  1,  1,  3,  1);
  srv(a150, b90, c150, d90, 42,  33,  33,  6,   1,  1,  1,  3);
  srv(a150, b90, c150, d90, 42,  33,  33,  42,  1,  1,  1,  3);
  srv(a180, b0,  c120, d60, 42,  6,   33,  42,  1,  3,  1,  1);
  //srv(a180, b0,  c120, d60, 42,  15,  33,  42,  1,  3,  1,  1);
  
}

//== Back ==================================================================================

void back ()
{
  // set servo positions and speeds needed to walk backward one step
  // (LFP,  LBP, RBP,  RFP, LFL, LBL, RBL, RFL, S1, S2, S3, S4)
  srv(180, 0,  120, 60, 42, 33, 33, 42, 3,  1, 1, 1);
  srv(150, 90, 150, 90, 42, 18, 33, 42, 1,  3, 1, 1);
  srv(150, 90, 150, 90, 42, 33, 33, 42, 1,  3, 1, 1);
  srv(120, 60, 180, 0,  42, 33, 33, 6,  1,  1, 1, 3);
  srv(120, 60, 180, 0,  42, 33, 33, 42, 1,  1, 1, 3);
  srv(90,  30, 90,  30, 42, 33, 18, 42, 1,  1, 3, 1);
  srv(90,  30, 90,  30, 42, 33, 33, 42, 1,  1, 3, 1);
  srv(180, 0,  120, 60, 6,  33, 33, 42, 3,  1, 1, 1);

}

//== Left =================================================================================

void turn_left()
{
  // set servo positions and speeds needed to turn left one step
  // (LFP,  LBP, RBP,  RFP, LFL, LBL, RBL, RFL, S1, S2, S3, S4)
  srv(150,  90, 90,  30, 42, 6,  33, 42, 1, 3, 1, 1);
  srv(150,  90, 90,  30, 42, 33, 33, 42, 1, 3, 1, 1);
  srv(120,  60, 180, 0,  42, 33, 6,  42, 1, 1, 3, 1);
  srv(120,  60, 180, 0,  42, 33, 33, 24, 1, 1, 3, 1);
  srv(90,   30, 150, 90, 42, 33, 33, 6,  1, 1, 1, 3);
  srv(90,   30, 150, 90, 42, 33, 33, 42, 1, 1, 1, 3);
  srv(180,  0,  120, 60, 6,  33, 33, 42, 3, 1, 1, 1);
  srv(180,  0,  120, 60, 42, 33, 33, 33, 3, 1, 1, 1);
}

//== Right ================================================================================

void turn_right()
{
  // set servo positions and speeds needed to turn right one step
  // (LFP,  LBP, RBP,  RFP, LFL, LBL, RBL, RFL, S1, S2, S3, S4)
  srv( 90, 30, 150, 90, 6,  33, 33, 42, 3, 1, 1, 1);
  srv( 90, 30, 150, 90, 42, 33, 33, 42, 3, 1, 1, 1);
  srv(120, 60, 180, 0,  42, 33, 33, 6,  1, 1, 1, 3);
  srv(120, 60, 180, 0,  42, 33, 33, 42, 1, 1, 1, 3);
  srv(150, 90, 90,  30, 42, 33, 6,  42, 1, 1, 3, 1);
  srv(150, 90, 90,  30, 42, 33, 33, 42, 1, 1, 3, 1);
  srv(180, 0,  120, 60, 42, 6,  33, 42, 1, 3, 1, 1);
  srv(180, 0,  120, 60, 42, 33, 33, 42, 1, 3, 1, 1);
}

//== Center Servos ========================================================================

void center_servos()
{
  servoFLPivot.write(a90);
  servoFLLift.write(servo_lift_center);
  servoBLPivot.write(b90);
  servoBLLift.write(servo_lift_center);
  servoBRPivot.write(c90);
  servoBRLift.write(servo_lift_center);
  servoFRPivot.write(d90);
  servoFRLift.write(servo_lift_center);
}

//== Change Speed ========================================================================

void change_speed(int delta)
{
  speed = speed + delta;

  // make sure speed stays within limits
  if (speed > SPEED_MAX)
  {
    speed = SPEED_MAX;
  }
  else if (speed < SPEED_MIN)
  {
    speed = SPEED_MIN;
  }
}

//== Change Height ========================================================================

void change_height(int delta)
{
  height = height + delta;

  // make sure speed stays within limits
  if (height > HEIGHT_MAX)
  {
    height = HEIGHT_MAX;
  }
  else if (height < HEIGHT_MIN)
  {
    height = HEIGHT_MIN;
  }
  
  recalculate_servo_centers();
  center_servos();
}


//== Recalculate Servo Center =================================================================

void recalculate_servo_centers()
{
    // Left Front Pivot
  a90 = (90 + da),
  a120 = (120 + da),
  a150 = (150 + da),
  a180 = (180 + da);

  // Left Back Pivot
  b0 = (0 + db),
  b30 = (30 + db),
  b60 = (60 + db),
  b90 = (90 + db);

  // Right Back Pivot
  c90 = (90 + dc),
  c120 = (120 + dc),
  c150 = (150 + dc),
  c180 = (180 + dc);

  // Right Front Pivot
  d0 = (0 + dd),
  d30 = (30 + dd),
  d60 = (60 + dd),
  d90 = (90 + dd);
  
  // find the lift servo center point, dependent on desired height
  // buffer is 25% of range, allowing 75% for movement 
  servo_lift_center = SERVO_LIFT_MIN + SERVO_LIFT_BUFFER + (int) (SERVO_LIFT_RANGE * 0.75 * (height) / HEIGHT_MAX);

}

//== Srv ===================================================================================

void srv( int  p11, int p21, int p31, int p41, int p12, int p22, int p32, int p42, int sp1, int sp2, int sp3, int sp4)
{

  // p11: Front Left Pivot Servo
  // p21: Back Left Pivot Servo
  // p31: Back Right Pivot Servo
  // p41: Front Right Pivot Servo
  // p12: Front Left Lift Servo
  // p22: Back Left Lift Servo
  // p32: Back Right Lift Servo
  // p42: Front Right Lift Servo
  // sp1: Speed 1
  // sp2: Speed 2
  // sp3: Speed 3
  // sp4: Speed 4
 
  // find current servo positions
  int s11 = servoFLPivot.read();
  int s12 = servoFLLift.read();
  int s21 = servoBLPivot.read();
  int s22 = servoBLLift.read();
  int s31 = servoBRPivot.read();
  int s32 = servoBRLift.read();
  int s41 = servoFRPivot.read();
  int s42 = servoFRLift.read();

  // Multiply lift servo positions by manual height adjustment
  p12 = p12 + high * 3;
  p22 = p22 + high * 3;
  p32 = p32 + high * 3;
  p42 = p42 + high * 3;

  // Higher speed => shorter delay
  // Lower speed  => longer delay
  int delayTime = 55 - 5 * speed; // ms

  while ((s11 != p11) || (s21 != p21) || (s31 != p31) || (s41 != p41) || (s12 != p12) || (s22 != p22) || (s32 != p32) || (s42 != p42))
  {

    // Front Left Pivot Servo
    if (s11 < p11)            // if servo position is less than programmed position
    {
      if ((s11 + sp1) <= p11)
        s11 = s11 + sp1;      // set servo position equal to servo position plus speed constant
      else
        s11 = p11;
    }

    if (s11 > p11)            // if servo position is greater than programmed position
    {
      if ((s11 - sp1) >= p11)
        s11 = s11 - sp1;      // set servo position equal to servo position minus speed constant
      else
        s11 = p11;
    }

    // Back Left Pivot Servo
    if (s21 < p21)
    {
      if ((s21 + sp2) <= p21)
        s21 = s21 + sp2;
      else
        s21 = p21;
    }

    if (s21 > p21)
    {
      if ((s21 - sp2) >= p21)
        s21 = s21 - sp2;
      else
        s21 = p21;
    }

    // Back Right Pivot Servo
    if (s31 < p31)
    {
      if ((s31 + sp3) <= p31)
        s31 = s31 + sp3;
      else
        s31 = p31;
    }

    if (s31 > p31)
    {
      if ((s31 - sp3) >= p31)
        s31 = s31 - sp3;
      else
        s31 = p31;
    }

    // Front Right Pivot Servo
    if (s41 < p41)
    {
      if ((s41 + sp4) <= p41)
        s41 = s41 + sp4;
      else
        s41 = p41;
    }

    if (s41 > p41)
    {
      if ((s41 - sp4) >= p41)
        s41 = s41 - sp4;
      else
        s41 = p41;
    }

    // Front Left Lift Servo
    if (s12 < p12)
    {
      if ((s12 + sp1) <= p12)
        s12 = s12 + sp1;
      else
        s12 = p12;
    }

    if (s12 > p12)
    {
      if ((s12 - sp1) >= p12)
        s12 = s12 - sp1;
      else
        s12 = p12;
    }

    // Back Left Lift Servo
    if (s22 < p22)
    {
      if ((s22 + sp2) <= p22)
        s22 = s22 + sp2;
      else
        s22 = p22;
    }

    if (s22 > p22)
    {
      if ((s22 - sp2) >= p22)
        s22 = s22 - sp2;
      else
        s22 = p22;
    }

    // Back Right Lift Servo
    if (s32 < p32)
    {
      if ((s32 + sp3) <= p32)
        s32 = s32 + sp3;
      else
        s32 = p32;
    }

    if (s32 > p32)
    {
      if ((s32 - sp3) >= p32)
        s32 = s32 - sp3;
      else
        s32 = p32;
    }

    // Front Right Lift Servo
    if (s42 < p42)
    {
      if ((s42 + sp4) <= p42)
        s42 = s42 + sp4;
      else
        s42 = p42;
    }

    if (s42 > p42)
    {
      if ((s42 - sp4) >= p42)
        s42 = s42 - sp4;
      else
        s42 = p42;
    }

    // Write Pivot Servo Values
    servoFLPivot.write(s11 + da);
    servoBLPivot.write(s21 + db);
    servoBRPivot.write(s31 + dc);
    servoFRPivot.write(s41 + dd);

    // Write Lift Servos Values
    servoFLLift.write(s12);
    servoBLLift.write(s22);
    servoBRLift.write(s32);
    servoFRLift.write(s42);

    delay(delayTime); // Delay before next movement

  }//while
} //srv

//== USRF Function ========================================================================

long get_distance(bool unit)
{
  // if unit == 0 return inches, else return cm

  long duration = 0, 
       cm = 0, 
       inches = 0;

  // The sensor is triggered by a HIGH pulse of 10 or more microseconds.
  // Give a short LOW pulse beforehand to ensure a clean HIGH pulse:
  digitalWrite(trigPin, LOW);
  delayMicroseconds(5);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // Read the signal from the sensor: a HIGH pulse whose
  // duration is the time (in microseconds) from the sending
  // of the ping to the reception of its echo off of an object.
  pinMode(echoPin, INPUT);
  duration = pulseIn(echoPin, HIGH);

  // convert the time into a distance
  cm = (duration / 2) / 29.1;
  inches = (duration / 2) / 74;

  if (unit == INCH)
    return inches;
  else
    return cm;
}

