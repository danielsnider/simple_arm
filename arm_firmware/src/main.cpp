#include <Arduino.h>
#include <Servo.h>
#define victorMax 2350
#define victorMin 650

struct JOINTPINS{
  int j0 = 9; // wrist roll
  int j1 = 10; // wrist pitch
  int j2 = 11; // small arm
  int j3 = 12;  // big arm
  int j4 = 13;  // base yaw
  int g1 = A4;  // gripper enable
  int g2 = A5;  // gripper open
  int g3 = A6;  // gripper close
  int w1 = A0;  // winch enable
  int w2 = A1;  // winch in
  int w3 = A2;  // winch out
  // int g1 = A0;  // gripper enable
  // int g2 = A1;  // gripper open
  // int g3 = A2;  // gripper close
  // int w1 = A4;  // winch enable
  // int w2 = A5;  // winch in
  // int w3 = A6;  // winch out
  int zedPan = 8; // camera
  int camTilt = 7; // camera
  // int p1 = A0; // unkown
  // int p2 = A1; // unkown
  // int p3 = A2; // pot3 aka. small arm
  // int p4 = A3; // pot4 aka. wrist pitch
}pins;

struct RECIEVED{
  float j0,j1,j2,j3,j4,gripper,winch,zedPan,camTilt;
}data;

// struct POTS{
//   int p1,p2,p3,p4;
// }potsdata;

Servo joint0, joint1, joint2, joint3, joint4, camTilt, zedPan;

int mapToVictor(float input){
  return map((input*100),-100,100,victorMin,victorMax);
}
void recievedData(){
  if(Serial.available()>=sizeof(float)){
    Serial.readBytes((char*)&data.gripper,sizeof(float));
    Serial.readBytes((char*)&data.j0,sizeof(float));
    Serial.readBytes((char*)&data.j1,sizeof(float));
    Serial.readBytes((char*)&data.j2,sizeof(float));
    Serial.readBytes((char*)&data.j3,sizeof(float));
    Serial.readBytes((char*)&data.j4,sizeof(float));
    Serial.readBytes((char*)&data.camTilt,sizeof(float));
  }
}
// void readPots(){
//   potsdata.p1 = analogRead(pins.p1);
//   potsdata.p2 = analogRead(pins.p2);
//   potsdata.p3 = analogRead(pins.p3);
//   potsdata.p4 = analogRead(pins.p4);
// }
// void sendData(){
//   Serial.write((char*)&potsdata.p1, sizeof(int));
//   Serial.write((char*)&potsdata.p2, sizeof(int));
//   Serial.write((char*)&potsdata.p3, sizeof(int));
//   Serial.write((char*)&potsdata.p4, sizeof(int));
// }
void writeToJoints(){
  joint0.writeMicroseconds(mapToVictor(data.j0));
  joint1.writeMicroseconds(mapToVictor(data.j1));
  joint2.writeMicroseconds(mapToVictor(data.j2));
  joint3.writeMicroseconds(mapToVictor(data.j3));
  joint4.writeMicroseconds(mapToVictor(data.j4));
  // zedPan.writeMicroseconds(mapToVictor(data.zedPan));
  zedPan.write((int)data.zedPan);
  camTilt.write((int)data.camTilt);

  // Gripper Open
  if (data.gripper == 1){
    digitalWrite(pins.g1, HIGH);   // enable pin
    digitalWrite(pins.g2, HIGH);   // turn open on
    digitalWrite(pins.g3, LOW);   // turn close off
  }
  // Gripper Close
  else if (data.gripper == -1){
    digitalWrite(pins.g1, HIGH);   // enable pin
    digitalWrite(pins.g2, LOW);   // turn open off
    digitalWrite(pins.g3, HIGH);   // turn close on
  }
  // Gripper Stop
  else if (data.gripper == 0){
    digitalWrite(pins.g1, LOW);   // enable pin
    digitalWrite(pins.g2, LOW);   // turn open off
    digitalWrite(pins.g3, LOW);   // turn close on
  }

  // Winch In
  if (data.winch == 1){
    digitalWrite(pins.w1, HIGH);   // enable pin
    digitalWrite(pins.w2, HIGH);   // turn open on
    digitalWrite(pins.w3, LOW);   // turn close off
  }
  // Winch Out
  else if (data.winch == -1){
    digitalWrite(pins.w1, HIGH);   // enable pin
    digitalWrite(pins.w2, LOW);   // turn open off
    digitalWrite(pins.w3, HIGH);   // turn close on
  }
  // Winch Off
  else if (data.winch == 0){
    digitalWrite(pins.w1, LOW);   // enable pin
    digitalWrite(pins.w2, LOW);   // turn open off
    digitalWrite(pins.w3, LOW);   // turn close off
  }
  // Winch Break
  else if (data.winch == 1337){
    digitalWrite(pins.w1, HIGH);   // enable pin
    digitalWrite(pins.w2, HIGH);   // turn open on
    digitalWrite(pins.w3, HIGH);   // turn close on
  }
}
void setup() {
  Serial.begin(9600);
  joint0.attach(pins.j0);
  joint1.attach(pins.j1);
  joint2.attach(pins.j2);
  joint3.attach(pins.j3);
  joint4.attach(pins.j4);
  zedPan.attach(pins.zedPan);
  camTilt.attach(pins.camTilt);
  // pinMode(pins.zedPan, OUTPUT);
  pinMode(pins.g1, OUTPUT);
  pinMode(pins.g2, OUTPUT);
  pinMode(pins.g3, OUTPUT);
  pinMode(pins.w1, OUTPUT);
  pinMode(pins.w2, OUTPUT);
  pinMode(pins.w3, OUTPUT);
  // pinMode(pins.p1, INPUT);
  // pinMode(pins.p2, INPUT);
  // pinMode(pins.p3, INPUT);
  // pinMode(pins.p3, INPUT);
  recievedData();
}

void loop() {
  // readPots();
  // sendData();
  recievedData();
  writeToJoints();
}