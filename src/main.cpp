#include "GPS_functions.h"

using namespace vex;
using namespace std;

void autonomous(void) {
  //runAuton();
  //oneminute310p();
}

void usercontrol(void) {
  
  while (1) {
    
    vexDelay(10);
  }
}

int main() {
  vexDelay(200);
  task GP1(positioning);
  task GP2(GPSpositioning);
  task GP(posConfig);

  vexDelay(200);
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  while (true) {
    vexDelay(100);
  }
}
