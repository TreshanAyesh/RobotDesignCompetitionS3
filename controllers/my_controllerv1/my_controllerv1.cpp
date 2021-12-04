#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/DistanceSensor.hpp>
#include <iostream>
#include <string>
#include <cmath>

using namespace webots;
using namespace std;

#define TIME_STEP 32
#define MAX_SPEED 5.0

//IR sensor reading value limit
float ir_limit = 30.0;

float left_speed=MAX_SPEED;
float right_speed=MAX_SPEED;

DistanceSensor *sensors[8]={};
string sensor_names[8]={"IR0","IR1","IR2","IR3","IR4","IR5","IR6","IR7"};
int readings[8]={};

//PID constants
const float Kp=21;
const float Ki=0.0;
const float Kd=0.1;
float last_error=0.0;
float error_sum=0.0;

//functions
void getReading();
void PID();
int count_num(int *array,int num);
void print_array(int *array);

Motor *left_motor;
Motor *right_motor;

int main(){
  Robot *robot = new Robot();
  
  //Motors
  left_motor = robot->getMotor("left wheel motor");
  right_motor = robot->getMotor("right wheel motor");  
  left_motor->setPosition(INFINITY);
  right_motor->setPosition(INFINITY);
  left_motor->setVelocity(0);
  right_motor->setVelocity(0);
  
  //Sensors
  for (int ir_num=0;ir_num<8;ir_num++){
    DistanceSensor *IR_sensor = robot->getDistanceSensor(sensor_names[ir_num]);
    sensors[ir_num]=IR_sensor;
    sensors[ir_num]->enable(TIME_STEP);
  }
   while (robot->step(TIME_STEP) != -1){
     getReading();
     print_array(readings);
     PID();     
   }
}

//get readings from IR sensors
void getReading(){
  for (int ir_num=0;ir_num<8;ir_num++){
    //float read = sensors[ir_num]->getValue();
    //cout<<read<<',';
    if (sensors[ir_num]->getValue() > ir_limit){
      readings[ir_num]=1;
    }
    else{
      readings[ir_num]=0;
    }
  }
}

void PID(){
  int count=count_num(readings,0);
  if (count!=8){
  float error= 0.0;
  float P,I,D;
  int coefficients[8]={-400,-300,-200,-100,100,200,300,400};
  
  for (int i=0;i<8;i++){
    error+=coefficients[i]*readings[i];
  }
  
  P = Kp*error;
  I = 0;//error_sum+Ki*error;
  D = (error-last_error)*Kd;  
  
  float correction = float((P+I+D))/10000;
  
  left_speed = MAX_SPEED*0.5+correction;
  right_speed = MAX_SPEED*0.5-correction;
  
  if (left_speed > MAX_SPEED) left_speed=MAX_SPEED;
  if (left_speed < 0) left_speed=0;
  if (right_speed > MAX_SPEED) right_speed=MAX_SPEED;
  if (right_speed < 0) right_speed=0;
  
  error_sum += error;
  last_error = error; 
  }
  
  
  cout<<left_speed<<','<<right_speed<<endl;
  
  
  //int count=count_num(readings,0);
  //if (count!=8){
    //cout<<"black"<<endl;
  //}
  
  left_motor->setVelocity(left_speed);
  right_motor->setVelocity(right_speed);
  
}

int count_num(int *array,int num){
  int count=0;
  for (int i=0;i<int(sizeof(array));i++){
    if (array[i]==num) count++;
  }
  return count;
}

void print_array(int *array){
  cout<<'[';
  for (int item=0;item<int(sizeof(array));item++){
    cout<<array[item]<<", ";
  }
  cout<<']';
}
