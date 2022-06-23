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

// Motorjh
motor LA = motor(PORT10, ratio18_1, false);
motor LB = motor(PORT8, ratio18_1, false);
motor RA = motor(PORT5, ratio18_1, false);
motor RB = motor(PORT9,  ratio18_1, false);

motor fw1 = motor(PORT6,  ratio6_1, true);
motor fw2 = motor(PORT7,  ratio6_1, true);

motor itk = motor(PORT4, ratio18_1, false);
motor ind = motor(PORT3, ratio18_1, false);

// Encoder
encoder Hor = encoder(Brain.ThreeWirePort.C);
encoder Ver = encoder(Brain.ThreeWirePort.A);

digital_out sol = digital_out(Brain.ThreeWirePort.H);

// Sensor
gps GPS = gps(PORT15, 0.00, 0.00, mm, 0.00);

inertial Gyro = inertial(PORT3);
gyro v4gyro = gyro(Brain.ThreeWirePort.G);

#endif