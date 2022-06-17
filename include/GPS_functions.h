#ifndef _ADVANCED_
#define _ADVANCED_

#undef __ARM_NEON__
#undef __ARM_NEON

#include "definitions_and_declarations.h"

Eigen::Vector2f rings(-5.35f, 10.6f);
Eigen::Vector2f fGoal(-13.5f, 34.0f);
Eigen::Vector2f bGoal(-15.0f, -15.0f);
Eigen::Vector2f gpsPos(-33.0f, -167.0f);
Eigen::Vector2f omniPos(-33.0f, -167.0f);
Eigen::Vector2f globalPos(0.0f, 0.0f);
Eigen::Vector2f DriftPos(-16.0f, 0.0f);
Eigen::Vector2f OmnitoGPS(-3.3f, 17.0f);
Eigen::Vector2f GET(-16.0f, 0.0f);

static float globalRot = PI / 2;
static float chasFacing = PI / 2;
bool driftFlag = true;

auto enc2cm = [](float enc, float wc) -> float { return enc * wc / 360; };
auto absAng = [](float ang) -> float { return ang-2*PI*floor((ang+PI)/(2*PI));};

void Rotate(float w){}

void speedForward(Eigen::Vector2f LR) {
  LA.spin(fwd, LR[0], vex::velocityUnits::rpm);
  LB.spin(fwd, LR[0], vex::velocityUnits::rpm);
  RB.spin(fwd, LR[0], vex::velocityUnits::rpm);
  RA.spin(fwd, LR[1], vex::velocityUnits::rpm);
}

class Chas {
public:
  Eigen::Vector2f encoderValues;
  Eigen::Vector2f localSpeed;
  Eigen::Vector2f lastEncoderValues;
  Eigen::Vector4f theta;

  float gyroValue;
  float lastGyro;
  float deltagyro;
  float insideHalfGyro;
  

public:
  Chas();
  void printInfo();
  void updateGlobalPos(float step, float i);
};

Chas chassis = Chas();

Chas::Chas() {
  encoderValues = Eigen::Vector2f::Zero();
  localSpeed = Eigen::Vector2f::Zero();
  lastEncoderValues = Eigen::Vector2f::Zero();
  theta = Eigen::Vector4f(PI / 2, PI / 2, PI / 2, PI / 2);

  gyroValue = PI / 2;
  lastGyro = PI / 2;
  deltagyro = 0;
  insideHalfGyro = 0;
}

void Chas::printInfo() {
  if (Controller1.ButtonA.pressing()) {
    Brain.Screen.printAt(10, 60, "eGyro:%f", theta[2]);
    Brain.Screen.printAt(10, 80, "gGyro:%f", theta[3]);
    Brain.Screen.printAt(10, 100, "trueGyro:%f", lastGyro);
    Brain.Screen.printAt(10, 120, "XPOS:%f", globalPos[0]);
    Brain.Screen.printAt(10, 140, "YPOS:%f", globalPos[1]);
    Brain.Screen.printAt(10, 160, "encoderH:%f", Hor.rotation(degrees));
    Brain.Screen.printAt(10, 180, "encoderV:%f", Ver.rotation(degrees));
  }
}

void Chas::updateGlobalPos(float step, float i) {

  insideHalfGyro =
      (lastGyro + (i / step) * deltagyro + (i / (2 * step)) * deltagyro) -
      PI / 2;

  Eigen::Matrix2f rotationMatrix;
  rotationMatrix << cos(insideHalfGyro), -sin(insideHalfGyro),
                    sin(insideHalfGyro),  cos(insideHalfGyro);

  omniPos += ((rotationMatrix * localSpeed) * (1 / step));
  if (omniPos.norm() > 400) {
    omniPos = Eigen::Vector2f::Zero();
  }
}

int positioning() {
  vexDelay(100);
  Gyro.resetRotation();
  v4gyro.resetRotation();
  Hor.resetRotation();
  Ver.resetRotation();
  float iter = 0, mark = 0, step = 0;

  while (1) {
    iter += 1;

    chassis.encoderValues <<-enc2cm(Hor.rotation(degrees), _SGL_OMNI_CIRCUM_CM_),
                            -enc2cm(Ver.rotation(degrees), _SGL_OMNI_CIRCUM_CM_);
    chassis.theta << chassis.theta[2], chassis.theta[3],
        v4gyro.rotation() * PI / 180 + PI / 2,
        -Gyro.rotation(degrees) * PI / 180 + PI / 2;
    chassis.localSpeed = chassis.encoderValues - chassis.lastEncoderValues;
    chassis.printInfo();

    vexDelay(1);
    if (chassis.localSpeed.norm() == 0)
      continue;
    step = iter - mark > 30 ? 30 : iter - mark;

    chassis.gyroValue =
        chassis.theta[1] == chassis.theta[3]
            ? chassis.theta[1] + chassis.theta[2] - chassis.theta[0]
            : chassis.theta[3];
    chassis.deltagyro = chassis.gyroValue - chassis.lastGyro;
    globalRot = chassis.gyroValue;

    for (int i = 0; i < step; i++) {
      chassis.updateGlobalPos(step, i);
    }

    chassis.lastEncoderValues = chassis.encoderValues;
    chassis.lastGyro = chassis.gyroValue;
    mark = iter;
  }
  return 0;
}

int GPSpositioning() {
  delay(100);
  GPS.calibrate();
  while (GPS.isCalibrating()) {delay(10);}
  while (true) {
    gpsPos << ( GPS.xPosition()) / 10, 
              ( GPS.yPosition()) / 10;
    delay(10);
  }
  return 0;
}

bool omnionly = true;
bool GPSDisable = false;

int posConfig() {
  delay(100);
  Eigen::Vector2f configVec(0.0f, 0.0f);
  Eigen::Matrix2f T; T.Zero();
  while (1) {
    T << sin(globalRot), cos(globalRot), -cos(globalRot), sin(globalRot);
    if (GPS.quality() > 99 && !GPSDisable) {
      configVec = configVec * 0.9 + (gpsPos - (omniPos + T * OmnitoGPS)) * 0.1;
    }
    globalPos =  omnionly ? omniPos : ((omniPos + T * OmnitoGPS) + configVec);
    delay(20);
  }
}

void drift(float driftX, float driftY, int tp = 1, float speedratio = 1.0f,
           float brakeRatio = 0.4) { // 0 for bottom, 1 for forward
  driftFlag = true;
  float yAxisCos = 0;
  const float posTol = 5;
  const float chassis_half_width = 19.2;

  Eigen::Vector2f driftTarget(driftX, driftY);
  Eigen::Vector2f localTarget(1000, 1000);
  Eigen::Vector2f yAxis(0, 1);
  Eigen::Vector2f globalTrackingPos = Eigen::Vector2f::Zero();
  Eigen::Vector2f localTrackingPos = Eigen::Vector2f::Zero();
  Eigen::Vector2f LR = Eigen::Vector2f::Zero();
  Eigen::Vector2f TLR = Eigen::Vector2f::Zero();

  switch (tp) {
  case 0: localTrackingPos = bGoal; break;
  case 1: localTrackingPos = fGoal; break;
  case 2: localTrackingPos = rings; break;
  }

  float omega = PI / 4 - 0.05;
  Eigen::Matrix2f MotionVector;
  MotionVector << cos(omega), -cos(omega), sin(omega), sin(omega);

  while (localTarget.norm() > posTol && driftFlag) {

    Eigen::Matrix2f T;
    T << sin(globalRot), cos(globalRot), -cos(globalRot), sin(globalRot);
    globalTrackingPos = globalPos + T * localTrackingPos;
    T << sin(globalRot), -cos(globalRot), cos(globalRot), sin(globalRot);
    localTarget = T * (driftTarget - globalTrackingPos);

    yAxisCos = localTarget.dot(yAxis) / (localTarget.norm() * yAxis.norm());
    if (0 && (yAxisCos < 0.3) && localTarget.norm() > 60) {
      LR << sign(localTarget[0]) * 120, -sign(localTarget[0]) * 120;
    } else if (localTarget[1] < 0 && localTarget.norm() < 8) {
      break;
    } else {
      LR = MotionVector.inverse() * localTarget;
      LR.normalize();
      float f = 1;
      if (localTarget.norm() < 100)
        f = brakeRatio + (localTarget.norm() * (1 - brakeRatio) * 0.01);
      LR *= (200 * speedratio * f) / max(abs(LR[0]), abs(LR[1]));
    }
    TLR << LR[1], LR[0];
    speedForward(tp == 0 ? TLR : LR);

    vexDelay(10);
  }
  speedForward(Eigen::Vector2f(0.0f, 0.0f));
  cout << localTarget << " " << speedratio << " " << brakeRatio << endl;

  return;
}

Eigen::Vector2f getPos(Eigen::Vector2f localPos) {
  Eigen::Matrix2f T;
  T << sin(globalRot), cos(globalRot), -cos(globalRot), sin(globalRot);
  return globalPos + T * localPos;
}

double lookAtCalc(double tarX, double tarY, Eigen::Vector2f currentPos) {
  double delX = tarX - currentPos[0], delY = tarY - currentPos[1];
  double absRot = delX == 0 ? PI / 2 : atan(delY / delX);
  if (delX > 0)
    absRot += PI;
  return absRot * 180 / PI;
}

void PIDGyroTurnDegLookAt(
    double tarX, double tarY, float kp = 2.8, float ki = 0.32,
    float kd = 6.8) { // the target parameter here should be the return value of
                      // the function lookAtCalc
  float timeroffset = TIMER;
  float timeused = 0;
  // float kp = 1, ki = 0, kd = 1;//kp = 2.9
  float irange = 50;
  float istart = 5;
  float dtol = 8;
  float errortolerance = 4;
  float lim = 100;
  float error = -1; // target - (getGyro() - floor(getGyro()/360)*360);
  double target = 0;
  float lasterror;
  float v = 0;
  float i = 0;
  bool arrived;
  float timetol = 2000; // float timetol = fabs(error) > 200 ? 1200 : 2000;
  float pow;
  lasterror = error;
  arrived = 0;
  // cout<<lookAtCalc(tarX, tarY, getPos(GET))<<"  "<<getPos(GET)[0]<<"
  // "<<getPos(GET)[1]<<"  "<<target - (getGyroLA() -
  // floor(getGyroLA()/360)*360)<<endl;
  while (!arrived) {
    target = lookAtCalc(tarX, tarY, getPos(GET));
    timeused = TIMER - timeroffset;
    error = target - (getGyroLA - floor(getGyroLA / 360) * 360);
    if (fabs(error) > 180)
      error = -sign(error) * (360 - fabs(error));
    v = error - lasterror;
    lasterror = error;
    if ((fabs(error) < errortolerance &&
         fabs(Gyro.gyroRate(zaxis, dps)) <= dtol) ||
        timeused > timetol) {
      arrived = true;
    }
    if (fabs(i) < irange && fabs(error) < istart)
      i += error;
    if (error * lasterror <= 0)
      i = 0;
    pow = kp * error + kd * v + ki * i;
    pow = fabs(pow) > lim ? sign(pow) * lim : pow;
    Rotate(-1 * pow);
    // cout<<error<<" "<<endl;
    delay(25);
  }
  // cout<<"break"<<endl;
  Rotate(0);
}

struct BP{
  int breakType;
  int param;
};
BP breakParam;
#define TBreak  0
#define XBreak  1
#define YBreak  2
#define DBreak  3

int breakDrift(void *args){
  int startTime = TIMER;
  bool breakCondition = true;
  BP *_param = (BP*)args;
  float Xerror = _param->param - globalPos[0], Yerror = _param->param - globalPos[1];
  while(breakCondition){
    switch(_param->breakType){
      case 0: breakCondition = (TIMER-startTime < _param->param); break;
      case 1: breakCondition = (Xerror * (_param->param - globalPos[0]) >0); break;
      case 2: breakCondition = (Yerror * (_param->param - globalPos[1]) >0); break;
      //case 3: breakCondition = (BRDis > _param->param); break;
    }
    delay(5);
  }
  driftFlag = false; return 0;
}


#endif