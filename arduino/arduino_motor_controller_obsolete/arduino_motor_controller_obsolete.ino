// Include necessary libraries
#include <ros.h>
#include <tank_mcu/arduinoCtrl.h>
#include <tank_mcu/arduinoData.h>
#include <tank_mcu/arduinoEstop.h>
#include <tank_mcu/arduinoPidTune.h>


// Define controls
#define LEFT 0
#define RIGHT 1
#define FORWARD 1
#define BACKWARD -1
#define STOP 0


// Define max and min PWM values
#define LOWER_BND 48
#define UPPER_BND 255


// Just for convenience
using namespace tank_mcu;


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


// Struct for setting and getting PI controller's parameters
struct PIparameters {
  float kp;
  float ki;
};


// Velocity PI controller class
class PIcontroller{
public:
  // Constructor
  PIcontroller(int motor_, float kp_, float ki_) : motor(motor_), params({kp_, ki_}), target(0.0f), V_filtered(0.0f), V_prev(0.0f), err_integ(0.0f) {
    time_prev = micros();  
  }
  
  // Get filtered velocity (in tick/ms)
  float getVelocity() {
    return V_filtered;
   }

  // Tune PI controller's parameters (Set new values if they are numbers. Otherwise just return current values)
  PIparameters tuneParameters(float kp_, float ki_){
    if (!isnan(kp_)) {
      params.kp = kp_;
    }
    if (!isnan(ki_)) {
      params.ki = ki_;
    }

    // C++ returns copies of standard variables, not references
    return {params.kp, params.ki};
  }

  // Function called in loop to control motor, returns filtered velocity (in ticks/ms)
  float doControl() {    // TODO - Automatically stop engines if no movement was detected for over 1s
    // Scale time to miliseconds, reset timer
    long time_curr = micros();
    float time_delta = float(time_curr - time_prev) / 1000.0f;
    time_prev = time_curr;

    // Get traveled distance, change it to current velocity, reset counter
    float V = float(deltaPos[motor])/ time_delta;
    deltaPos[motor] = 0;

    // Low pass filter
    V_filtered = 0.8818f * V_filtered + 0.0591f * V + 0.0591f * V_prev;
    V_prev = V;

    if (globalStop) {
      // Set remaining variables to 0 and stop motors
      target = 0.0f;
      err_integ = 0.0f;
      motorCtrl(motor, STOP, 0);
    } else {
      // Calculate velocity error
      float err = target - V_filtered;

      // Calculate integral, and contrain it to prevent overflow
      // (Yes, I've managed to overflow a float. In fact this controller can do this twice a second if left without constraint)
      err_integ = constrain(err_integ + (err * time_delta), -2496.0f, 2496.0f);

      // Calculate control value
      float u = params.kp * err + params.ki * err_integ;  
      
      // Separate and scale control; send data to control function
      unsigned int val = fabs(u);
      motorCtrl(motor, u >= 0 ? FORWARD : BACKWARD, val < LOWER_BND ? 0 : (val > UPPER_BND ? UPPER_BND : val));
    }

    // Return current velocity
    return V_filtered;
  }

private:
  int8_t motor;
  PIparameters params;

  long time_prev;
  float V_filtered;
  float V_prev;
  float err_integ;

public:
  volatile float target;
};


// PI controller class objects
PIcontroller controller[2] = { {PIcontroller(LEFT, 30, 0.3)}, 
                               {PIcontroller(RIGHT, 30, 0.3)} };


// Callback function called whenever new controls are received                               
void update_targets(const arduinoCtrl& ctrl) {
  controller[LEFT].target = ctrl.left_motor;
  controller[RIGHT].target = ctrl.right_motor;
}


// Callback function for enabling/disabling global stop
void update_globalstop(const arduinoEstop::Request& req, arduinoEstop::Response& res) {
  if (req.estop == 1) { 
    globalStop = true; 
  } else {
    globalStop = false;
  }
}


// Callback function for tuning PI controller's parameters
void update_pidtune(const arduinoPidTune::Request& req, arduinoPidTune::Response& res) {
  PIparameters ret = controller[LEFT].tuneParameters(req.set_k, req.set_ti);
  controller[RIGHT].tuneParameters(req.set_k, req.set_ti);
  res.get_k = ret.kp;
  res.get_ti = ret.ki;
}


// ROS objects
ros::NodeHandle nh;
arduinoData data_msg;
ros::Publisher send_data("arduino_data", &data_msg);
ros::Subscriber<arduinoCtrl> receive_ctrl("arduino_ctrl", &update_targets);
ros::ServiceServer<arduinoEstop::Request, arduinoEstop::Response> estop("arduino_estop", &update_globalstop);
ros::ServiceServer<arduinoPidTune::Request, arduinoPidTune::Response> pid_tune("arduino_pid_tune", &update_pidtune);


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

    // Initialize ROS
    nh.initNode();
    nh.advertise(send_data);
    nh.advertiseService(estop);
    nh.advertiseService(pid_tune);
    nh.subscribe(receive_ctrl);
}


// Main program loop
void loop() {
  data_msg.estop = globalStop ? 1 : 0;
  data_msg.left_motor = controller[LEFT].doControl();
  data_msg.right_motor = controller[RIGHT].doControl();

  send_data.publish(&data_msg);
  nh.spinOnce();
}
