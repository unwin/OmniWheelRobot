// OmniBot.pde
//
// Midlevel control routines for a stepper motor based omni wheeled robot where the wheels are aranged in a "+" pattern
//
// Author Roger Unwin (2013)
/* 
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include <AccelStepper.h>
#include <math.h>

#define ENABLE A0
#define SLEEP  A1
#define RESET  A2
#define MS3    A3
#define MS2    A4
#define MS1    A5
#define pi 3.1415926535897932384626433
#define BASE_STEPS_PER_REV 200
#define MAX_SPEED 9600
#define MAX_ACCEL 500
#define MIN_PULSE 20

AccelStepper stepper0(AccelStepper::DRIVER, 7, 2);
AccelStepper stepper1(AccelStepper::DRIVER, 8, 3);
AccelStepper stepper2(AccelStepper::DRIVER, 4, 9);
AccelStepper stepper3(AccelStepper::DRIVER, 5, 10);

int _index_ = 0;
char _line_[80];

char _step_ = 0b001;
char _sleep_ = true;
char _enable_ = false;
double _max_speed_ = MAX_SPEED;
double _max_accel_ = MAX_ACCEL;
double _wheel_diameter_  = 10.4775; // CM
double _base_diameter_  = 47.625; // CM
 
String stepping_table[8];
int steps_per_rev[8];

void setup()
{  
  Serial.begin(115200);
  
  stepping_table[0] = "Full Step";
  steps_per_rev[0] = BASE_STEPS_PER_REV;
  stepping_table[1] = "Half Step";
  steps_per_rev[1] = 2 * BASE_STEPS_PER_REV;
  stepping_table[2] = "Quarter Step";
  steps_per_rev[2] = 4 * BASE_STEPS_PER_REV;
  stepping_table[3] = "Eighth Step";
  steps_per_rev[3] = 8 * BASE_STEPS_PER_REV;
  stepping_table[4] = "UNDEFINED Using Full Step";
  steps_per_rev[4] = 0;
  stepping_table[5] = "UNDEFINED Using Full Step";
  steps_per_rev[5] = 0;
  stepping_table[6] = "UNDEFINED Using Full Step";
  steps_per_rev[6] = 0;
  stepping_table[7] = "Sisteenth Step";
  steps_per_rev[7] = 16 * BASE_STEPS_PER_REV;
  
  pinMode(ENABLE, OUTPUT);
  pinMode(SLEEP, OUTPUT);
  pinMode(RESET, OUTPUT);
  pinMode(MS3, OUTPUT);
  pinMode(MS2, OUTPUT);
  pinMode(MS1, OUTPUT);
  
  reset_driver();
  set_step(_step_);
  set_max_speed(_max_speed_);
  set_max_accel(_max_accel_);
  set_min_pulse_width(MIN_PULSE);
  zero_location();
  disable_steppers();
  sleep();
}

void loop()
{
  if (Serial.available()) {
    _line_[_index_] = Serial.read();
    if ((_line_[_index_] == 10) || (_line_[_index_] == 13)) {
      _line_[_index_] = 0;
      _index_ = 0;
      Serial.print("---------------\nREAD: ");
      Serial.println(_line_);
      Serial.println("===============");
      
      if (0 == strcmp(_line_, "status")) {
        display_status();
      } else if (0 == strncmp(_line_, "moveto", 6)) {
        int i = 0;
        for (i = 6;((_line_[i] != ',') && (_line_[i] != 0));i++) ;
        _line_[i] = 0;
        i++;
        double r = atol(&_line_[6]);
        double theta = atol(&_line_[i]);

        moveto(r, theta);
      } else if (0 == strcmp(_line_, "stop")) {
        stop_all();
      } else if (0 == strcmp(_line_, "hardstop")) {
        hard_stop_all();
      } else if (0 == strncmp(_line_, "sleep", 5)) {
        sleep();
      } else if (0 == strncmp(_line_, "wake", 4)) {
        wake();
      } else if (0 == strncmp(_line_, "enable", 6)) {
        enable_steppers();
      } else if (0 == strncmp(_line_, "disable", 7)) {
        disable_steppers();
      } else if (0 == strncmp(_line_, "maxspeed", 8)) {
        set_max_speed(atol(&_line_[8]));
      } else if (0 == strncmp(_line_, "maxaccel", 8)) {
        set_max_accel(atol(&_line_[8]));
      } else if (0 == strcmp(_line_, "reset")) {
        reset_driver();
      } else if (0 == strncmp(_line_, "step", 4)) {
        set_step(char(atol(&_line_[4])) & 0b111);
      } else if (0 == strncmp(_line_, "wheel_diameter", 14)) {
        set_wheel_diameter(atol(&_line_[15]));
      } else if (0 == strncmp(_line_, "base_diameter", 13)) {
        set_base_diameter(atol(&_line_[14]));
      } else if (0 == strncmp(_line_, "rotate", 6)) {
        rotate(atol(&_line_[6]));
      } 
    } else {
      _index_++;
    }
  }
    
  stepper0.run();
  stepper1.run();
  stepper2.run();
  stepper3.run();
}

void display_status() {
  Serial.print("SLEEP: ");
  Serial.println(_sleep_, DEC);
  Serial.print("ENABLE: ");
  Serial.println(_enable_, DEC);
  Serial.print("OMNI WHEEL DIAMETER: ");
  Serial.print(_wheel_diameter_);
  Serial.println("cm");
  Serial.print("BASE DIAMETER: ");
  Serial.print(_base_diameter_);
  Serial.println("cm");
  Serial.print("step = ");       
  Serial.println(stepping_table[_step_]);
  Serial.print("steps per rev = ");
  Serial.println(steps_per_rev[_step_]);
  Serial.print("max speed = ");
  Serial.println(_max_speed_); 
}

void set_base_diameter(double diameter) {
  _base_diameter_ = diameter;
  Serial.print("setting base diameter to ");
  Serial.print(_base_diameter_);
  Serial.println("cm");
}

void set_wheel_diameter(double diameter) {
  _wheel_diameter_ = diameter;
  Serial.print("setting omni wheel diameter to ");
  Serial.print(_wheel_diameter_);
  Serial.println("cm");
}

void set_step(int step) {
  _step_ = step;
  if (step & 0b100)
    digitalWrite(MS3, HIGH);
  else
    digitalWrite(MS3, LOW); 
    
  if (step & 0b010)
    digitalWrite(MS2, HIGH);
  else
    digitalWrite(MS2, LOW); 
    
  if (step & 0b001)
    digitalWrite(MS1, HIGH);
  else
    digitalWrite(MS1, LOW); 
    
  Serial.print("step = ");     
  Serial.println(stepping_table[_step_]);
  Serial.print("steps per rev = ");
  Serial.println(steps_per_rev[_step_]);
}

void set_max_speed(double speed) {
  _max_speed_ = speed;
  stepper0.setMaxSpeed(speed);
  stepper1.setMaxSpeed(speed);
  stepper2.setMaxSpeed(speed);
  stepper3.setMaxSpeed(speed);
  
  Serial.print("max speed set to ");
  Serial.println(speed); 
}

void set_max_accel(double accel) {
  _max_accel_ = accel;
  stepper0.setAcceleration(accel);
  stepper1.setAcceleration(accel);
  stepper2.setAcceleration(accel);
  stepper3.setAcceleration(accel);
  
  Serial.print("max accel set to ");
  Serial.println(accel);
}

void set_min_pulse_width(double pulse_width) {
  stepper0.setMinPulseWidth(pulse_width);
  stepper1.setMinPulseWidth(pulse_width);
  stepper2.setMinPulseWidth(pulse_width);
  stepper3.setMinPulseWidth(pulse_width);
}

void enable_steppers() {
  digitalWrite(ENABLE, LOW);
  _enable_ = true;
}

void disable_steppers() {
  digitalWrite(ENABLE, HIGH);
  _enable_ = false;
}

void sleep() {
  digitalWrite(SLEEP, LOW);
  _sleep_ = true;
}

void wake() {
  digitalWrite(SLEEP, HIGH);
  _sleep_ = false;
}

void zero_location() {
  stepper0.setCurrentPosition(0);
  stepper1.setCurrentPosition(0);
  stepper2.setCurrentPosition(0);
  stepper3.setCurrentPosition(0);
}

void reset_driver() {
  digitalWrite(RESET, LOW);
  delay(100);
  digitalWrite(RESET, HIGH);
}

void hard_stop_all() {
  stepper0.setCurrentPosition(stepper0.targetPosition());
  stepper1.setCurrentPosition(stepper1.targetPosition());
  stepper2.setCurrentPosition(stepper2.targetPosition());
  stepper3.setCurrentPosition(stepper3.targetPosition());
}

void stop_all() {
  stepper0.stop();
  stepper1.stop();
  stepper2.stop();
  stepper3.stop();
}

void moveto(double r, double theta) {
    double theta_rad = pi / 180.0 * theta;
    double x = r * cos(theta_rad);
    double y = r * sin(theta_rad);
    double wheel_circumference = pi * _wheel_diameter_;
    double cm_per_step = wheel_circumference / steps_per_rev[_step_];
    double x_steps = x / cm_per_step;
    double y_steps = y / cm_per_step;

    if (abs(x_steps) > abs(y_steps)) {
      double slow_percent = abs(y_steps / x_steps); 
      double slow_speed = slow_percent * _max_accel_;
      stepper0.setAcceleration(_max_accel_);
      stepper1.setAcceleration(_max_accel_);
      stepper2.setAcceleration(slow_speed);
      stepper3.setAcceleration(slow_speed); 
    } else if (abs(x_steps) < abs(y_steps)) {
      double slow_percent = abs(x_steps / y_steps); 
      double slow_speed = slow_percent * _max_accel_;
      stepper0.setAcceleration(slow_speed);
      stepper1.setAcceleration(slow_speed);
      stepper2.setAcceleration(_max_accel_);
      stepper3.setAcceleration(_max_accel_); 
    } else
      set_max_accel(_max_accel_);
    
    stepper0.move(x_steps);
    stepper1.move(-x_steps);
    stepper2.move(y_steps);
    stepper3.move(-y_steps);
}

void rotate(double deg) {
    set_max_accel(_max_accel_);
    double base_circumference = pi * _base_diameter_;
    double wheel_circumference = pi * _wheel_diameter_;
    double cm_per_step = wheel_circumference / steps_per_rev[_step_];
    double steps = ((deg / 360.0) * base_circumference) / cm_per_step;
    
    stepper0.move(steps);
    stepper1.move(steps);
    stepper2.move(steps);
    stepper3.move(steps);
}
