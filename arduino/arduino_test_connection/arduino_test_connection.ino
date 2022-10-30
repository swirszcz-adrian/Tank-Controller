 // Include ATOMIC_BLOCK macro
#include <util/atomic.h>


// Define controls
#define LEFT 0
#define RIGHT 1
#define FORWARD 1
#define BACKWARD -1
#define STOP 0


// Define max and min PWM values
#define LOWER_BND 51
#define UPPER_BND 255


// Define max velocity value that can be send via serial port
#define MAX_VELOCITY 9.9


// Define wait time, after which board will assume serial connection was lost (in microseconds)
#define CONNECTION_LOST_TIME 2000000U


// Define motor control pins
const int inApin[2] = {7, 4};   // INA: Clockwise input
const int inBpin[2] = {8, 9};   // INB: Counter-clockwise input
const int pwmpin[2] = {5, 6};   // PWM


// Define encoder pins
const int encPower[2] = {12, 13};   // Encoders power
const int encA[2] = {2, 3};         // First encoder
const int encB[2] = {10, 11};       // Second encoder


// Variable for counting encoder ticks, that occured between program loops
volatile int deltaPos[2] = {0, 0};


// Variable for stopping robot
volatile bool globalStop = true;


// Variable for recording valid and bad messages
volatile unsigned int good_msgs;
volatile unsigned int bad_msgs;


// Functions used to write data from encoders (two different functions since engines rotate in different directions)
void readLeftEncoder(){
  if (digitalRead(encB[LEFT]) == LOW) { deltaPos[LEFT]++; }
  else { deltaPos[LEFT]--; }
}
void readRightEncoder(){
  if (digitalRead(encB[RIGHT]) == HIGH) { deltaPos[RIGHT]++; }
  else { deltaPos[RIGHT]--; }
}


// Program used to control individual motors, no comments inside since it's self-explanatory
void motorCtrl(int8_t motor, int8_t direct, uint8_t value) {
  switch(motor) {
    case LEFT:
      switch (direct) {
        case FORWARD:
            digitalWrite(inApin[LEFT], LOW);
            digitalWrite(inBpin[LEFT], HIGH);
            analogWrite(pwmpin[LEFT], value);
          break;
        case BACKWARD:
            digitalWrite(inApin[LEFT], HIGH);
            digitalWrite(inBpin[LEFT], LOW);
            analogWrite(pwmpin[LEFT], value);
          break;
        case STOP:
            digitalWrite(inApin[LEFT], LOW);
            digitalWrite(inBpin[LEFT], LOW);
            analogWrite(pwmpin[LEFT], 0);
          break;
        default:
          ;
        }
      break;
    
    case RIGHT:
      switch (direct) {
        case FORWARD:
            digitalWrite(inApin[RIGHT], HIGH);
            digitalWrite(inBpin[RIGHT], LOW);
            analogWrite(pwmpin[RIGHT], value);
          break;
        case BACKWARD:
            digitalWrite(inApin[RIGHT], LOW);
            digitalWrite(inBpin[RIGHT], HIGH);
            analogWrite(pwmpin[RIGHT], value);
          break;
        case STOP:
            digitalWrite(inApin[RIGHT], LOW);
            digitalWrite(inBpin[RIGHT], LOW);
            analogWrite(pwmpin[RIGHT], 0);
          break;
        default:
          ;
        }
      break;
    
    default:
      ;
  }
}


// Velocity PI controller class
class PIcontroller{
public:
  // Constructor
  PIcontroller(int motor, float kp, float ki) : motor_(motor), kp_(kp), ki_(ki), target_(0.0f), V_filtered_(0.0f), V_prev_(0.0f), err_integ_(0.0f) {
    time_prev_ = micros();  
  }
  
  // Get filtered velocity (in tick/ms)
  float getVelocity() {
    return V_filtered_;
   }

  // Interface for reading/writing controller's kp_ parameter
  float getKp() {
    return kp_;
  }
  void setKp(float kp) {
    kp_ = kp;
  }

  // Interface for reading/writing controller's ki_ parameter
  float getKi() {
    return ki_;
  }
  void setKi(float ki) {
    ki_ = ki;
  }
  
  // Function called in loop to control motor, returns filtered velocity (in ticks/ms)
  float doControl() {
    // Scale time to miliseconds, reset timer
    long time_curr = micros();
    float time_delta = float(time_curr - time_prev_) / 1000.0f;
    time_prev_ = time_curr;

    // Get traveled distance, reset counter
    int distance;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    // code with interrupts blocked (consecutive atomic operations will not get interrupted)
      distance = deltaPos[motor_];
      deltaPos[motor_] = 0;
    }

    // Change distance to velocity
    float V = float(distance) / time_delta;

    // Low pass filter
    V_filtered_ = 0.22826f * V_filtered_ + 0.38587f * V + 0.38587f * V_prev_;
    V_prev_ = V;

    if (globalStop) {
      // Set remaining variables to 0 and stop motors
      target_ = 0.0f;
      err_integ_ = 0.0f;
      motorCtrl(motor_, STOP, 0);
      
    } else {
      // Calculate velocity error
      float err = target_ - V_filtered_;

      // Calculate integral, and contrain it to prevent overflow
      // (Yes, I've managed to overflow a float. In fact this controller can do this twice a second if left without constraint)
      err_integ_ = constrain(err_integ_ + (err * time_delta), -2496.0f, 2496.0f);

      // Calculate control value
      float u = kp_ * err + ki_ * err_integ_;  
      
      // Separate and scale control; send data to control function
      unsigned int val = fabs(u);
      motorCtrl(motor_, u >= 0 ? FORWARD : BACKWARD, val < LOWER_BND ? 0 : (val > UPPER_BND ? UPPER_BND : val));
    }

    // Return current velocity
    return V_filtered_;
  }

private:
  int8_t motor_;
  float kp_;
  float ki_;
  long time_prev_;
  float V_filtered_;
  float V_prev_;
  float err_integ_;

public:
  volatile float target_;
};


// PI controller class objects
PIcontroller controller[2] = { {PIcontroller(LEFT, 30, 0.3)}, 
                               {PIcontroller(RIGHT, 30, 0.3)} };


// Receive commands from USB and execute them
// This function gets called automatically by Arduino at the end of loop if data is available
void serialEvent() {
  while (Serial.available() >= 5) {
    // Check if message was valid
    bool received_msg = false;
    
    String full_string = Serial.readStringUntil('\n');
    full_string.trim();
    
    // First 2 letters dictate type of command. The remaining part (after excluding the separator) is treated as a command-specific value
    String command = full_string.substring(0, 2);
    String value = full_string.substring(3);

    // Execute actions corresponding to given command (sadly without switch since it requires variable to be of int type)
    if (command == String("MV") && value.length() > 10) {
      // Set new target value for left and right PI controllers
      // Example value: "+3.14 -0.21"
      controller[LEFT].target_ = value.substring(0, 5).toFloat();
      controller[RIGHT].target_ = value.substring(6, 11).toFloat();
      if (controller[LEFT].target_ == 2.0f && controller[RIGHT].target_ == 2.0f) { received_msg = true; }
      
    } else if (command == String("ST")) {
      // Switch globalStop to true or false
      // Example value: "OFF"
      if (value == String("OFF")) {
        globalStop = false;
      } else if (value == String("ON")) {
        globalStop = true;
      }
      received_msg = true;
      
    } else if (command == String("PL")) {
      // If provided update left PI controller's kp and ki values
      // Always print current parameteres of left controller 
      // Example value: "GET" or "30.00 00.30" 
      if (value.length() > 10) {
        controller[LEFT].setKp(value.substring(0, 5).toFloat());
        controller[LEFT].setKi(value.substring(6, 11).toFloat());
      } 
      float kp = controller[LEFT].getKp();
      float ki = controller[LEFT].getKi();
      String msg = String("PL ");
      msg += (kp < 10.0f ? String("0") + String(kp, 2) : String(kp, 2));
      msg += (ki < 10.0f ? String(" 0") + String(ki, 2) : String(" ") + String(ki, 2));
      Serial.println(msg);
      received_msg = true;
      
    } else if (command == String("PR")) {
      // If provided update right PI controller's kp and ki values
      // Always print current parameteres of right controller 
      // Example value: "GET" or "30.00 00.30" 
      if (value.length() > 10) {
        controller[RIGHT].setKp(value.substring(0, 5).toFloat());
        controller[RIGHT].setKi(value.substring(6, 11).toFloat());
      }
      float kp = controller[RIGHT].getKp();
      float ki = controller[RIGHT].getKi();
      String msg = String("PR ");
      msg += (kp < 10.0f ? String("0") + String(kp, 2) : String(kp, 2));
      msg += (ki < 10.0f ? String(" 0") + String(ki, 2) : String(" ") + String(ki, 2));
      Serial.println(msg);
      received_msg = true;
    }

    // Update message counters
    if (received_msg) { 
      good_msgs++; 
    } else {
      bad_msgs++;
    }
  }
}


// Initialization
void setup() {
  for (int i=0; i<2; i++) {
    // Initialize digital pins as outputs
    pinMode(inApin[i], OUTPUT);
    pinMode(inBpin[i], OUTPUT);
    pinMode(pwmpin[i], OUTPUT);
    pinMode(encPower[i], OUTPUT);

    // Initialize digital pins as input
    pinMode(encA[i], INPUT);
    pinMode(encB[i], INPUT);

    // Make sure engines dont run at the beggining
    digitalWrite(inApin[i], LOW);
    digitalWrite(inBpin[i], LOW);
    analogWrite(pwmpin[i], 0);

    // Power up encoders
    digitalWrite(encPower[i], HIGH);
  }
    
  // Attach interrupts
  attachInterrupt(digitalPinToInterrupt(encA[LEFT]), readLeftEncoder, RISING);
  attachInterrupt(digitalPinToInterrupt(encA[RIGHT]), readRightEncoder, RISING);

  // Initialize serial communication
  Serial.begin(9600);
}


// Main program loop
void loop() {
  // Do control
  float trashcan;
  trashcan = controller[LEFT].doControl();
  trashcan = controller[RIGHT].doControl();

  // Prepeare serial message
  char buffer[15];
  char f1 = globalStop ? 'S' : 'M';
  char f2 = globalStop ? 'T' : 'V';
  sprintf(buffer, "%c%c %05d %05d", f1, f2, good_msgs, bad_msgs);

  // Send serial message
  Serial.println(buffer);
}
