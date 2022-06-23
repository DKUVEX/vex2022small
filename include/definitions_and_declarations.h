#ifndef DEFINITIONS_H
#define DEFINITIONS_H

#include "interfaces_and_devices.h"
#include <iostream>
using namespace std;

#define TIMER Brain.timer(vex::timeUnits::msec)
#define MyTimer Brain.Timer.value()
#define printScreen Brain.Screen.printAt
#define printController Controller1.Screen.print
#define clearController Controller1.Screen.clearLine
#define clearControllerAll Controller1.Screen.clearScreen

#define PI 3.141592653589793238462
#define _MID_OMNI_CIRCUM_CM_ 26.0
#define _SGL_OMNI_CIRCUM_CM_ 22.36

#define Ch1 Controller1.Axis1.position(percent)
#define Ch2 Controller1.Axis2.position(percent)
#define Ch3 Controller1.Axis3.position(percent)
#define Ch4 Controller1.Axis4.position(percent)

#define BA Controller1.ButtonA.pressing()
#define BB Controller1.ButtonB.pressing()
#define BX Controller1.ButtonX.pressing()
#define BY Controller1.ButtonY.pressing()

#define L1 Controller1.ButtonL1.pressing()
#define L2 Controller1.ButtonL2.pressing()
#define R1 Controller1.ButtonR1.pressing()
#define R2 Controller1.ButtonR2.pressing()

#define UP Controller1.ButtonUp.pressing()
#define DOWN Controller1.ButtonDown.pressing()
#define LEFT Controller1.ButtonLeft.pressing()
#define RIGHT Controller1.ButtonRight.pressing()

#define IDis            ((LDis+RDis)/2)
#define sign(x)         (x==0?0:(x>0?1:-1))
#define deg2rad(x)      (x/180.0f*PI)
#define rad2deg(x)      (x*180.0f/PI)
#define getGyro         (-Gyro.rotation())
#define delay           vexDelay
#define getGyroRad      (deg2rad(getGyro)+3.14)
#define getGyroLA       (getGyro-90)
#define resetGyro()     Gyro.resetRotation()
#define getliftEncoder  (Lift1.rotation(deg))

#define fw_OFF          0
#define fw_LSPD         1
#define fw_HSPD         2
#define fw_FSPD         3
#define fw_AUTO         4
#define fw_CONT         5

#define ctrl_DEFAULT    0
#define ctrl_MANUAL1    1
#define ctrl_MANUAL2    2
#define ctrl_AUTOAIM    3

static int fwState = fw_OFF;
static int chState = ctrl_DEFAULT;

static bool ifSpeedOK = false;

static float fw_angBias = 0.0565f;

Eigen::Vector2f blueGoal(-132.0f, 132.0f);
Eigen::Vector2f redGoal(132.0f, -132.0f);


#endif