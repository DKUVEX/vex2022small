#ifndef API_H
#define API_H

#include <iostream>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "v5.h"
#include "v5_vcs.h"
#include <eigen-3.4.0/Eigen/Dense>

using namespace vex;
using signature = vision::signature;
using code = vision::code;

brain Brain;
controller Controller1 = controller(primary);
competition Competition;

// Motor
motor LA = motor(PORT14, ratio18_1, true);
motor LM = motor(PORT13, ratio18_1, false);
motor LB = motor(PORT12, ratio18_1, true);
motor RA = motor(PORT3, ratio18_1, false);

// Encoder
encoder Hor = encoder(Brain.ThreeWirePort.G);
encoder Ver = encoder(Brain.ThreeWirePort.C);

// Sensor
gps GPS = gps(PORT9, 0.00, 0.00, mm, 0.00);

inertial Gyro = inertial(PORT15);
gyro v4gyro = gyro(Brain.ThreeWirePort.F);

#endif