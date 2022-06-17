#ifndef TASKS_H
#define TASKS_H

#include "GPS_functions.h"

void fw(float speed){
  fw1.spin(fwd, 120*speed, voltageUnits::mV);
  fw2.spin(fwd, 120*speed, voltageUnits::mV);
}

void intake(float speed){
  itk.spin(fwd, 120*speed, voltageUnits::mV);
}

void index(float speed){
  ind.spin(fwd, 120*speed, voltageUnits::mV);
}

int base(){
  bool manualMode = true, lastBB = false;

  float absGlbRot = PI / 2, angLock = PI / 2;
  Eigen::Vector2f facingVector; facingVector.Zero();// robot-goal vector
  Eigen::Vector2f chnl34Vector; chnl34Vector.Zero(); // ori ch3-4 input
  Eigen::Matrix2f T; T.Zero();

  while(1){
    if(BB && !lastBB) {
      manualMode = !manualMode;
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

    if(manualMode){
      LA.spin(fwd, 100*(Ch1+Ch3+Ch4), voltageUnits::mV);
      LB.spin(fwd, 100*(Ch1+Ch3-Ch4), voltageUnits::mV);
      RA.spin(fwd, 100*(Ch1-Ch3+Ch4), voltageUnits::mV);
      RB.spin(fwd, 100*(Ch1-Ch3-Ch4), voltageUnits::mV);
    }
    else{
      LA.spin(fwd, 100*( chnl34Vector[0] + chnl34Vector[1] - 200*absAng(chasFacing-globalRot)), voltageUnits::mV);
      LB.spin(fwd, 100*(-chnl34Vector[0] + chnl34Vector[1] - 200*absAng(chasFacing-globalRot)), voltageUnits::mV);
      RA.spin(fwd, 100*( chnl34Vector[0] - chnl34Vector[1] - 200*absAng(chasFacing-globalRot)), voltageUnits::mV);
      RB.spin(fwd, 100*(-chnl34Vector[0] - chnl34Vector[1] - 200*absAng(chasFacing-globalRot)), voltageUnits::mV);
    }

    printScreen(10,40,"%f",chnl34Vector[0]);
    delay(10);
    lastBB = BB;
  }

  return 0;
}


#endif