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

    switch(chState){
      case ctrl_DEFAULT:{
        LA.spin(fwd, 100*(Ch1+Ch3), voltageUnits::mV);//+Ch4
        LB.spin(fwd, 100*(Ch1+Ch3), voltageUnits::mV);//-Ch4
        RA.spin(fwd, 100*(Ch1-Ch3), voltageUnits::mV);//+Ch4
        RB.spin(fwd, 100*(Ch1-Ch3), voltageUnits::mV);//-Ch4
        break;
      }
      case ctrl_MANUAL1:{

        break;
      }
      case ctrl_MANUAL2:{

        break;
      }
      case ctrl_AUTOAIM:{
        LA.spin(fwd, 100*( chnl34Vector[0] + chnl34Vector[1] - 200*absAng(chasFacing-globalRot-PI)), voltageUnits::mV);
        LB.spin(fwd, 100*(-chnl34Vector[0] + chnl34Vector[1] - 200*absAng(chasFacing-globalRot-PI)), voltageUnits::mV);
        RA.spin(fwd, 100*( chnl34Vector[0] - chnl34Vector[1] - 200*absAng(chasFacing-globalRot-PI)), voltageUnits::mV);
        RB.spin(fwd, 100*(-chnl34Vector[0] - chnl34Vector[1] - 200*absAng(chasFacing-globalRot-PI)), voltageUnits::mV);
        break;
      }
    }
    delay(10);
    lastBX = BX;
  }

  return 0;
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
    fwSpeed = fwlastSpeed*0.8f + (fw1.velocity(velocityUnits::rpm)+fw2.velocity(velocityUnits::rpm))*0.2f*5.0f/2.0f;
    fwBaseSpeed = fwDistance - fwLastDistance;//backward speed

    //printScreen(10,60,"%f",fwSpeed);
    cout << fwSpeed << " " << fwVolt << endl;
    fwlastSpeed = fwSpeed;
    fwLastDistance = fwDistance;

    switch(fwState){
      case fw_OFF:{
        fw(0);
        break;
      }
      case fw_LSPD:{
        //2350
        fwTargetSpeed = 2135;

        ifSpeedOK = fwSpeed > (fwTargetSpeed-70) && fwSpeed < (fwTargetSpeed+70);
        if(ifFwPID && fwSpeed < (fwTargetSpeed-50)) ifFwPID = false;
        else if(!ifFwPID && fwSpeed > fwTargetSpeed-10) ifFwPID = true;
        fwVolt = ifFwPID?((67)+(fwTargetSpeed-fwSpeed)*0.1):100;
        fw(fwVolt);
        
      
        break;
      }
      case fw_HSPD:{
        fwTargetSpeed = 2775;

        ifSpeedOK = fwSpeed > (fwTargetSpeed-5) && fwSpeed < (fwTargetSpeed+5);
        if(ifFwPID && fwSpeed < (fwTargetSpeed-30)) ifFwPID = false;
        else if(!ifFwPID && fwSpeed > (fwTargetSpeed-2)) ifFwPID = true;
        fwVolt = ifFwPID?((90)+(fwTargetSpeed-fwSpeed)*0.15):100;
        fw(fwVolt);
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


#endif