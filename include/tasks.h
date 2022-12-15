#ifndef TASKS_H
#define TASKS_H

#include "GPS_functions.h"
void fw(float speed){
  if(speed < 0){
    fw1.spin(fwd, 0, voltageUnits::mV);
    fw2.spin(fwd, 0, voltageUnits::mV);
  }
  else{
    fw1.spin(fwd, 120*speed, voltageUnits::mV);
    fw2.spin(fwd, 120*speed, voltageUnits::mV);
  }
}

void intake(float speed){
  itk.spin(fwd, 120*speed, voltageUnits::mV);
}

void index(float speed){
  ind.spin(fwd, 120*speed, voltageUnits::mV);
}
void roller(float speed){
  rol.spin(fwd, 120*speed, voltageUnits::mV);
} 

void extend(float speed){
  ext.spin(fwd, 120*speed, voltageUnits::mV);
}
void kick(int ktime){
  for(int i=0;i<ktime;i++){
    sol.set(1);
    vexDelay(100);
    sol.set(0);
    vexDelay(220);
  }  
}

int base(){
  bool lastBX = false;
  
  float absGlbRot = PI / 2, angLock = PI / 2;
  Eigen::Vector2f facingVector; facingVector.Zero();// robot-goal vector
  Eigen::Vector2f chnl34Vector; chnl34Vector.Zero(); // ori ch3-4 input
  Eigen::Matrix2f T; T.Zero();

  while(1){
    if(BX && !lastBX) {
      chState = (chState==3)?0:chState+1;
      angLock = globalRot;
    }
    facingVector << 132-globalPos[0], -132-globalPos[1];
    chasFacing = facingVector[0]>0 ? 
      atan(facingVector[1]/facingVector[0]) :
      atan(facingVector[1]/facingVector[0]) - PI;

    chnl34Vector << Ch4, Ch3;
    absGlbRot = absAng(globalRot);
    T << cos(absGlbRot-PI/2), sin(absGlbRot-PI/2), -sin(absGlbRot-PI/2), cos(absGlbRot-PI/2);
    chnl34Vector = T * chnl34Vector;

    //cout << chState << endl;

    switch(chState){
      case ctrl_DEFAULT:{
        LA.spin(fwd, 100*(Ch1+Ch3), voltageUnits::mV);
        LB.spin(fwd, 100*(Ch1+Ch3), voltageUnits::mV);
        RA.spin(fwd, 100*(Ch1-Ch3), voltageUnits::mV);
        RB.spin(fwd, 100*(Ch1-Ch3), voltageUnits::mV);
        break;
      }
      case ctrl_MANUAL1:{
        LA.spin(fwd, 100*(Ch1+Ch3+Ch4), voltageUnits::mV);
        LB.spin(fwd, 100*(Ch1+Ch3-Ch4), voltageUnits::mV);
        RA.spin(fwd, 100*(Ch1-Ch3+Ch4), voltageUnits::mV);
        RB.spin(fwd, 100*(Ch1-Ch3-Ch4), voltageUnits::mV);
        break;
      }
      case ctrl_MANUAL2:{
        LA.spin(fwd, 100*(Ch1+Ch3), voltageUnits::mV);//+Ch4
        LB.spin(fwd, 100*(Ch1+Ch3), voltageUnits::mV);//-Ch4
        RA.spin(fwd, 100*(Ch1-Ch3), voltageUnits::mV);//+Ch4
        RB.spin(fwd, 100*(Ch1-Ch3), voltageUnits::mV);//-Ch4
        break;
      }
      case ctrl_AUTOAIM:{
        fwState = fw_AUTO;
        LA.spin(fwd, 100*( chnl34Vector[0] + chnl34Vector[1] - 200*absAng(chasFacing-globalRot-PI)), voltageUnits::mV);
        LB.spin(fwd, 100*(-chnl34Vector[0] + chnl34Vector[1] - 200*absAng(chasFacing-globalRot-PI)), voltageUnits::mV);
        RA.spin(fwd, 100*( chnl34Vector[0] - chnl34Vector[1] - 200*absAng(chasFacing-globalRot-PI)), voltageUnits::mV);
        RB.spin(fwd, 100*(-chnl34Vector[0] - chnl34Vector[1] - 200*absAng(chasFacing-globalRot-PI)), voltageUnits::mV);
        //cout << chasFacing << endl;
        break;
      }
      case ctrl_AUTONOMOUS:{
        delay(100);
        break;
      }
    }
    lastBX = BX;
    delay(10);
  }

  return 0;
}
int speed_factor = 5;
void mov_fwd(int time)
{
  if (time>=200) 
  {
    for (int i=0;i<=100;i++)
    {
      LA.spin(fwd, speed_factor*10*i, voltageUnits::mV);
      LB.spin(fwd, speed_factor*10*i, voltageUnits::mV);
      RA.spin(fwd, -speed_factor*10*i, voltageUnits::mV);
      RB.spin(fwd, -speed_factor*10*i, voltageUnits::mV); 
    }
    LA.spin(fwd, speed_factor*1000, voltageUnits::mV);
    LB.spin(fwd, speed_factor*1000, voltageUnits::mV);
    RA.spin(fwd, -speed_factor*1000, voltageUnits::mV);
    RB.spin(fwd, -speed_factor*1000, voltageUnits::mV);  
    delay(time-200);
    for (int i=100;i>=0;i--)
    {
      LA.spin(fwd, speed_factor*10*i, voltageUnits::mV);
      LB.spin(fwd, speed_factor*10*i, voltageUnits::mV);
      RA.spin(fwd, -speed_factor*10*i, voltageUnits::mV);
      RB.spin(fwd, -speed_factor*10*i, voltageUnits::mV); 
    }
  }
  else
  {
    for (int i=0;i<=time/2;i++)
    {
      LA.spin(fwd, speed_factor*10*i, voltageUnits::mV);
      LB.spin(fwd, speed_factor*10*i, voltageUnits::mV);
      RA.spin(fwd, -speed_factor*10*i, voltageUnits::mV);
      RB.spin(fwd, -speed_factor*10*i, voltageUnits::mV); 
    }
    for (int i=time/2;i>=0;i--)
    {
      LA.spin(fwd, speed_factor*10*i, voltageUnits::mV);
      LB.spin(fwd, speed_factor*10*i, voltageUnits::mV);
      RA.spin(fwd, -speed_factor*10*i, voltageUnits::mV);
      RB.spin(fwd, -speed_factor*10*i, voltageUnits::mV); 
    }
  }
  LA.spin(fwd, 0, voltageUnits::mV);
  LB.spin(fwd, 0, voltageUnits::mV);
  RA.spin(fwd, 0, voltageUnits::mV);
  RB.spin(fwd, 0, voltageUnits::mV);  
}
void mov_bwd(int time)
{
  LA.spin(fwd, -10000, voltageUnits::mV);
  LB.spin(fwd, -10000, voltageUnits::mV);
  RA.spin(fwd, 10000, voltageUnits::mV);
  RB.spin(fwd, 10000, voltageUnits::mV);  
  delay(time);
  LA.spin(fwd, 0, voltageUnits::mV);
  LB.spin(fwd, 0, voltageUnits::mV);
  RA.spin(fwd, 0, voltageUnits::mV);
  RB.spin(fwd, 0, voltageUnits::mV);  
}
double rotate_left_factor = 5.1;
void rotate_left(int time)
{
  LA.spin(fwd, -rotate_left_factor*1000, voltageUnits::mV);
  LB.spin(fwd, -rotate_left_factor*1000, voltageUnits::mV);
  RA.spin(fwd, -rotate_left_factor*1000, voltageUnits::mV);
  RB.spin(fwd, -rotate_left_factor*1000, voltageUnits::mV);  
  delay(time);
  LA.spin(fwd, 1000, voltageUnits::mV);
  LB.spin(fwd, 1000, voltageUnits::mV);
  RA.spin(fwd, 1000, voltageUnits::mV);
  RB.spin(fwd, 1000, voltageUnits::mV);  
  delay(100);
  LA.spin(fwd, 0, voltageUnits::mV);
  LB.spin(fwd, 0, voltageUnits::mV);
  RA.spin(fwd, 0, voltageUnits::mV);
  RB.spin(fwd, 0, voltageUnits::mV);  
}
double rotate_right_factor = 5.1;
void rotate_right(int time)
{
  LA.spin(fwd, rotate_right_factor*1000, voltageUnits::mV);
  LB.spin(fwd, rotate_right_factor*1000, voltageUnits::mV);
  RA.spin(fwd, rotate_right_factor*1000, voltageUnits::mV);
  RB.spin(fwd, rotate_right_factor*1000, voltageUnits::mV);  
  delay(time);
  LA.spin(fwd, -1000, voltageUnits::mV);
  LB.spin(fwd, -1000, voltageUnits::mV);
  RA.spin(fwd, -1000, voltageUnits::mV);
  RB.spin(fwd, -1000, voltageUnits::mV);  
  delay(100);
  LA.spin(fwd, 0, voltageUnits::mV);
  LB.spin(fwd, 0, voltageUnits::mV);
  RA.spin(fwd, 0, voltageUnits::mV);
  RB.spin(fwd, 0, voltageUnits::mV);  
}

/*orientation = -1 is left, *orientation = 1 is right, */
void keep_rotate(int orientation)
{
  int rotation_factor = orientation*5000;
  LA.spin(fwd, rotation_factor, voltageUnits::mV);
  LB.spin(fwd, rotation_factor, voltageUnits::mV);
  RA.spin(fwd, rotation_factor, voltageUnits::mV);
  RB.spin(fwd, rotation_factor, voltageUnits::mV);  
}
void keep_forward()
{
  LA.spin(fwd, 10000, voltageUnits::mV);  
  LB.spin(fwd, 10000, voltageUnits::mV);
  RA.spin(fwd, -10000, voltageUnits::mV);
  RB.spin(fwd, -10000, voltageUnits::mV);  
}
// void keep_rotate_right()
// {
//   LA.spin(fwd, 10000, voltageUnits::mV);
//   LB.spin(fwd, 10000, voltageUnits::mV);
//   RA.spin(fwd, 10000, voltageUnits::mV);
//   RB.spin(fwd, 10000, voltageUnits::mV);  
// }
void stop_action()
{
  LA.spin(fwd, 0, voltageUnits::mV);
  LB.spin(fwd, 0, voltageUnits::mV);
  RA.spin(fwd, 0, voltageUnits::mV);
  RB.spin(fwd, 0, voltageUnits::mV);  
}

/*caculate the difference angle between current robot direction and target direction, at degree unit*/
double turn_to_at_theta(double target_x, double target_y)
{
  double angle = atan2((target_x - xpos), (target_y - ypos));
  double theta = - angle*(180/M_PI);
  return theta;
}
/*rotate the robot at a specific angle use gps*/
void rotate_angle(double theta)
{
  while (theta <= yaw_orientation-5||theta >= yaw_orientation+5)
  {
    if(theta>yaw_orientation)
    {
        keep_rotate(-1);
    }
    else if (theta<yaw_orientation)
    {
        keep_rotate(1);
    }
  }
  stop_action();
}
/*move the robot at a specific distence use gps*/
void move_to_position(double target_x, double target_y)
{
  while (xpos<=target_x-10||xpos>target_x+10)
  {

  }
}


float shotPos2time(float dis){
  //distance unit: cm
  //time unit: ms
  return dis*2.5f+200;
}

float shotPos2fwSpeed(float dis){
  //distance unit: cm
  //fwSpeed unit: /
  return 4*dis+1655;
}

float fwPower2fwSpeed(){

  return 0;
}

int flywheelContorl(){
  Eigen::Vector2f goal; goal = redGoal;
  float fwSpeedTarget = 0, fwEncoder = 0, fwLastEncoder = 0, fwSpeed = 0, fwlastSpeed = 0, fwVolt = 0, fwTargetSpeed = 0, 
        fwDistance = (goal - globalPos).norm(), fwLastDistance = 0, fwBaseSpeed = 0;
  float predictShotDis = 0;
  float predictShotSpd = 0;
  bool ifFwPID = false;
  while(1){
    fwDistance = (goal - globalPos).norm();
    fwSpeed = fwlastSpeed*0.95f + (fw1.velocity(velocityUnits::rpm)+fw2.velocity(velocityUnits::rpm))*0.05f*5.0f/2.0f;
    fwBaseSpeed = fwDistance - fwLastDistance;//backward speed

    if(ifSpeedOK){ printScreen(10,100,"OK");} else  printScreen(10,100,"    ");
    printScreen(10,120,"%f   ", fwSpeed);
    //cout << fwSpeed << " " << fwVolt << endl;
    fwlastSpeed = fwSpeed;
    fwLastDistance = fwDistance;

    switch(fwState){
      case fw_OFF:{
        fw(0);
        break;
      }
      case fw_LSPD:{
        //2350
        fwTargetSpeed = 1950; //1950 

        ifSpeedOK = fwSpeed > (fwTargetSpeed-150) && fwSpeed < (fwTargetSpeed+50);
        if(ifFwPID && fwSpeed < (fwTargetSpeed-50)) ifFwPID = false;
        else if(!ifFwPID && fwSpeed > fwTargetSpeed-10) ifFwPID = true;
        fwVolt = ifFwPID?((68)+(fwTargetSpeed-fwSpeed)*0.1):100;
        fw(DOWN?68:fwVolt);
        
      
        break;
      }
      case fw_HSPD:{
        fwTargetSpeed = 2775; //2775

        ifSpeedOK = fwSpeed > (fwTargetSpeed-5) && fwSpeed < (fwTargetSpeed+5);
        if(ifFwPID && fwSpeed < (fwTargetSpeed-30)) ifFwPID = false;
        else if(!ifFwPID && fwSpeed > (fwTargetSpeed-2)) ifFwPID = true;
        fwVolt = ifFwPID?((90)+(fwTargetSpeed-fwSpeed)*0.15):100;
        fw(DOWN?90:fwVolt);
        break;
      }
      case fw_FSPD:{

        break;
      }
      case fw_AUTO:{
        
        fwDistance = (goal - globalPos).norm();
        predictShotDis = fwDistance + 100*fwBaseSpeed;
        predictShotSpd = shotPos2fwSpeed(predictShotDis + (shotPos2time(predictShotDis)-100)*fwBaseSpeed);
        fwTargetSpeed = predictShotSpd;

        ifSpeedOK = fwSpeed > (fwTargetSpeed-35) && fwSpeed < (fwTargetSpeed+35);
        if(ifFwPID && fwSpeed < (fwTargetSpeed-30)) ifFwPID = false;
        else if(!ifFwPID && fwSpeed > (fwTargetSpeed-2)) ifFwPID = true;
        fwVolt = ifFwPID?((90)+(fwTargetSpeed-fwSpeed)*0.2):100;
        fw(fwVolt);
        
        break;
      }
      case fw_CONT:{
        fwTargetSpeed = 2135;

        //ifSpeedOK = fwSpeed > (fwTargetSpeed-70) && fwSpeed < (fwTargetSpeed+70);
        if(ifFwPID && fwSpeed < (fwTargetSpeed-50)) ifFwPID = false;
        else if(!ifFwPID && fwSpeed > fwTargetSpeed-50) ifFwPID = true;
        fwVolt = ifFwPID?((67)+(fwTargetSpeed-fwSpeed)*0.5):100;
        fw(fwVolt);
        break;
      }
    }

    delay(20);
  }
  return 0;
}

int launch(){
  bool lastL1 = false;
  while(1){
    //if(L1 && !lastL1 && kCount < 3) kCount+=1;
    //if(L2 && L1) kCount = 3;
    if(L1 && !lastL1) kCount += 3;
    if(!L1 && lastL1) kCount -= 2;
    if(kCount > 3) kCount = 3;
    if(kCount < 0) kCount = 0;
    lastL1 = L1;
    vexDelay(10);
  }
  return 0;
}



#endif