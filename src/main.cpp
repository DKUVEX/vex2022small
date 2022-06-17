#include "tasks.h"

using namespace vex;
using namespace std;

void autonomous(void) {
  //runAuton();
  //oneminute310p();
}

void usercontrol(void) {
  bool lastBX = false, lastr1 = false, lastr2 = false, lastl1 = false, lastl2 = false;
  bool lastUP = false, lastDOWN = false, lastBB = false;

  int flywheel = 0;
  float fwSpeedTarget = 0, fwEncoder = 0, fwLastEncoder = 0, fwSpeed = 0;
  sol.set(0);
  while (1) {
    if(L1 && !lastl1 && flywheel<=99) flywheel += 5;
    if(L2 && !lastl2 && flywheel>=1) flywheel -=5;
    printScreen(10,20,"%d",flywheel);

    if(BB && !lastBB) {
      sol.set(1);
      vexDelay(100);
      sol.set(0);

    }
    /*
  
    fw1.spin(fwd, flywheel, velocityUnits::pct);
    fw2.spin(fwd, flywheel, velocityUnits::pct);

    
    */
    fwEncoder = (fw1.rotation(rotationUnits::deg) + fw2.rotation(rotationUnits::deg))/2;
    fwSpeed = fwEncoder - fwLastEncoder;
    /* 
    if(UP) fwSpeedTarget = 100;
    if(DOWN) fwSpeedTarget = 70;

    if(fwSpeed < 0.85*fwSpeedTarget) fw(100);
    else {
      if(fwSpeedTarget == 100) fw(100);
      if(fwSpeedTarget == 70 ) fw(72);
    }
    */
    fw(-flywheel);
    intake(100*(R2-R1));
    
    printScreen(10,60,"%f",fwSpeed);

    if(BX && !lastBX) sol.set(!sol.value());

    lastBX = BX;
    lastr1 = R1;
    lastr2 = R2;
    lastl1 = L1;
    lastl2 = L2;
    lastBB = BB;
    fwLastEncoder = fwEncoder;
    vexDelay(10);
  }
}

int main() {
  vexDelay(200);
  task GP1(positioning);
  task GP2(GPSpositioning);
  task GP(posConfig);
  task BS(base);
  vexDelay(200);
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  while (true) {
    vexDelay(100);
  }
}
