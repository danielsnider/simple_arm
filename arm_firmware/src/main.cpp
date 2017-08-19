#include <Arduino.h>
#include <Servo.h>

// PWM specs of the Victor SP motor controller.
//      https://www.vexrobotics.com/217-9090.html
#define victorMax 2350
#define victorMin 650

struct JOINTPINS{
  int wrist_roll = 9; // wrist roll pin
  int wrist_pitch = 10; // wrist pitch pin
  int upper_elbow = 11; // upper elbow pin
  int lower_elbow = 12;  // lower elbow pin
  int base_yaw = 13;  // base yaw pin
  int grip_enable = A4;  // gripper enable pin
  int grip_open = A5;  // gripper open pin
  int grip_close = A6;  // gripper close pin
  int cam_tilt = 7; // camera servo pin
}pins;

struct RECIEVED{
  float wrist_roll,wrist_pitch,upper_elbow,lower_elbow,base_yaw,gripper,winch,cam_tilt;
}data;

Servo joint0, joint1, joint2, joint3, joint4, cam_tilt;

int mapToVictor(float input){
  return map((input*100),-100,100,victorMin,victorMax);
}
void recievedData(){
  if(Serial.available()>=sizeof(float)){
    Serial.readBytes((char*)&data.gripper,sizeof(float));
    Serial.readBytes((char*)&data.wrist_roll,sizeof(float));
    Serial.readBytes((char*)&data.wrist_pitch,sizeof(float));
    Serial.readBytes((char*)&data.upper_elbow,sizeof(float));
    Serial.readBytes((char*)&data.lower_elbow,sizeof(float));
    Serial.readBytes((char*)&data.base_yaw,sizeof(float));
    Serial.readBytes((char*)&data.cam_tilt,sizeof(float));
  }
}

void writeToJoints(){
  joint0.writeMicroseconds(mapToVictor(data.wrist_roll));
  joint1.writeMicroseconds(mapToVictor(data.wrist_pitch));
  joint2.writeMicroseconds(mapToVictor(data.upper_elbow));
  joint3.writeMicroseconds(mapToVictor(data.lower_elbow));
  joint4.writeMicroseconds(mapToVictor(data.base_yaw));
  cam_tilt.write((int)data.cam_tilt);

  // Gripper Open
  if (data.gripper == 1){
    digitalWrite(pins.grip_enable, HIGH);   // enable pin on
    digitalWrite(pins.grip_open, HIGH);   // turn open on
    digitalWrite(pins.grip_close, LOW);   // turn close off
  }
  // Gripper Close
  else if (data.gripper == -1){
    digitalWrite(pins.grip_enable, HIGH);   // enable pin on
    digitalWrite(pins.grip_open, LOW);   // turn open off
    digitalWrite(pins.grip_close, HIGH);   // turn close on
  }
  // Gripper Stop
  else if (data.gripper == 0){
    digitalWrite(pins.grip_enable, LOW);   // enable pin off
    digitalWrite(pins.grip_open, LOW);   // turn open off
    digitalWrite(pins.grip_close, LOW);   // turn close on
  }
}
void setup() {
  Serial.begin(9600);
  joint0.attach(pins.wrist_roll);
  joint1.attach(pins.wrist_pitch);
  joint2.attach(pins.upper_elbow);
  joint3.attach(pins.lower_elbow);
  joint4.attach(pins.base_yaw);
  cam_tilt.attach(pins.cam_tilt);
  pinMode(pins.grip_enable, OUTPUT);
  pinMode(pins.grip_open, OUTPUT);
  pinMode(pins.grip_close, OUTPUT);
  recievedData();
}

void loop() {
  recievedData();
  writeToJoints();
}