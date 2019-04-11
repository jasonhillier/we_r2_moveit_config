/* Purpose: This sketch uses ROS as well as MultiStepper, AccelStepper, and Servo libraries to control the 
 * BCN3D Moveo robotic arm. In this setup, a Ramps 1.4 shield is used on top of an Arduino Mega 2560.  
 * Subscribing to the following ROS topics: 1) joint_steps, 2) gripper_angle
 *    1) current_joint_pos is computed from the simulation in PC and sent Arduino via rosserial.  It contains
 *       the steps (computed from absolute joint angle) necessary for each motor to move to reach the goal position.
 *    2) gripper_angle contains the necessary gripper angle to grasp the object when the goal state is reached 
 * 
 * Publishing to the following ROS topics: joint_positions
 *    1) joint_positions is a topic used by move_it to show the current robot position.
 *       
 * Code modified from:
 * Author: Jesse Weisberg
 *  https://github.com/jesseweisberg/moveo_ros
 */
#if (ARDUINO >= 100)
  #include <Arduino.h>
#else
  #include <WProgram.h>
#endif
#include <ros.h>

#include <sensor_msgs/JointState.h>
#include <Servo.h> 
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <math.h>
#include <std_msgs/Int16.h>
#include <std_msgs/UInt16.h>
#include <AccelStepper.h>
#include <MultiStepper.h>

#define STEPS_PER_REV      7280
#define STEPPER_MAX_SPEED  1200
#define JOINT_PUBLISH_RATE  200 //ms
#define MOTOR_COUNT          5

// Joint 1
#define E1_STEP_PIN        36
#define E1_DIR_PIN         34
#define E1_ENABLE_PIN      30

// Joint 2
#define Z_STEP_PIN         46
#define Z_DIR_PIN          48
#define Z_ENABLE_PIN       62
#define Z_MIN_PIN          18
#define Z_MAX_PIN          19

// Joint 3
#define Y_STEP_PIN         60
#define Y_DIR_PIN          61
#define Y_ENABLE_PIN       56
#define Y_MIN_PIN          14
#define Y_MAX_PIN          15

// Joint 4
#define X_STEP_PIN         54
#define X_DIR_PIN          55
#define X_ENABLE_PIN       38

// Joint 5 
#define E0_STEP_PIN        26
#define E0_DIR_PIN         28
#define E0_ENABLE_PIN      24

AccelStepper joint0(1,E1_STEP_PIN, E1_DIR_PIN);
AccelStepper joint1(1,Z_STEP_PIN, Z_DIR_PIN);
AccelStepper joint2(1,Y_STEP_PIN, Y_DIR_PIN);
AccelStepper joint3(1,X_STEP_PIN, X_DIR_PIN);
AccelStepper joint4(1, E0_STEP_PIN, E0_DIR_PIN);

Servo gripper;
MultiStepper steppers;

long _last_publish_millis = 0;
long _current_joint_pos[MOTOR_COUNT] = {0};
char* _joint_names[MOTOR_COUNT];
volatile bool _joint_status = 0;

ros::NodeHandle nh;

//publisher to give joint feedback to ROS
sensor_msgs::JointState _joint_state_msg;
ros::Publisher joint_pub("/joint_positions",&_joint_state_msg);

//callbacks
void arm_cb(const sensor_msgs::JointState& joint_angles){
    _joint_status = 1;
    for(int i=0; i<joint_angles.position_length && i<MOTOR_COUNT; i++)
    {
        _current_joint_pos[i] = joint_angles.position[i]*STEPS_PER_REV;
    }
}

void gripper_cb( const std_msgs::UInt16& cmd_msg){
  gripper.write(cmd_msg.data); // Set servo angle, should be from 0-180  
  //digitalWrite(13, HIGH-digitalRead(13));  // Toggle led  
}

//instantiate subscribers
ros::Subscriber<sensor_msgs::JointState> arm_sub("/move_group/fake_controller_joint_positions",arm_cb); //subscribes to move_it fake controller, which publishes joint positions for motion plan.
ros::Subscriber<std_msgs::UInt16> gripper_sub("gripper_angle", gripper_cb); //subscribes to gripper position
//to publish from terminal: rostopic pub gripper_angle std_msgs/UInt16 <0-180>

void setup() {
  //put your setup code here, to run once:
  Serial.begin(57600);
  pinMode(13,OUTPUT);
  digitalWrite(13, HIGH); //confirm sketch loaded
  _joint_status = 0;

  nh.initNode();
  nh.subscribe(arm_sub);
  nh.subscribe(gripper_sub);
  //nh.advertise(steps);

  // Configure each stepper
  joint0.setMaxSpeed(STEPPER_MAX_SPEED);
  joint1.setMaxSpeed(STEPPER_MAX_SPEED);
  joint2.setMaxSpeed(STEPPER_MAX_SPEED);
  joint3.setMaxSpeed(STEPPER_MAX_SPEED);
  joint4.setMaxSpeed(STEPPER_MAX_SPEED);

  // Then give them to MultiStepper to manage
  steppers.addStepper(joint0);
  steppers.addStepper(joint1);
  steppers.addStepper(joint2);
  steppers.addStepper(joint3);
  steppers.addStepper(joint4);

  //build joint name list
  for(int i=0; i<MOTOR_COUNT; i++)
  {
      _joint_names[i] = new char[7];
      String("joint" + i).toCharArray(_joint_names[i], 7);
  }

  // Configure gripper servo
  gripper.attach(11);
  
  digitalWrite(13, LOW); //toggle led
}

void loop() {
    if (_joint_status == 1) // If command callback (arm_cb) is being called, execute stepper command
    {
        digitalWrite(13, HIGH); //have led indicate it is moving
        _joint_status = 0;

        steppers.moveTo(_current_joint_pos);
        nh.spinOnce();
        steppers.runSpeedToPosition(); // Blocks until all are in position

        digitalWrite(13, LOW); //have led indicate it is no longer moving
    }
    if (millis()-_last_publish_millis > JOINT_PUBLISH_RATE)
    {
        _last_publish_millis = millis();

        _joint_state_msg.name_length = MOTOR_COUNT;
        _joint_state_msg.name = _joint_names;
        _joint_state_msg.position_length = MOTOR_COUNT;
        _joint_state_msg.position[0] = joint0.currentPosition();
        _joint_state_msg.position[1] = joint1.currentPosition();
        _joint_state_msg.position[2] = joint2.currentPosition();
        _joint_state_msg.position[3] = joint3.currentPosition();
        _joint_state_msg.position[4] = joint4.currentPosition();
        _joint_state_msg.header.stamp = nh.now();

        joint_pub.publish(&_joint_state_msg);
    }

    nh.spinOnce();
    delay(1);
  
}
