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
  
  sol.set(0);
  while (1) {
    /*
    LA.spin(fwd, -1300, voltageUnits::mV);
    LB.spin(fwd, 1300, voltageUnits::mV);
    RA.spin(fwd, 1300, voltageUnits::mV);
    RB.spin(fwd, -1300, voltageUnits::mV);
    */

    if(L1 && L2) fwState = fw_OFF;
    else if(L1) fwState = fw_HSPD;
    else if(L2) fwState = fw_LSPD;

    //cout << BB << endl;


    if((fwState == fw_LSPD) && L2 && BB && !lastBB && ifSpeedOK){
      fwState = fw_CONT;
      kick(3);
      fwState = fw_LSPD;
    }
    else if(BB && !lastBB && ifSpeedOK) {
      kick(1);
    }

    intake(100*(R2-R1));
    index(100*BA);

    //if(BX && !lastBX) sol.set(!sol.value());

    lastBX = BX;
    lastr1 = R1;
    lastr2 = R2;
    lastl1 = L1;
    lastl2 = L2;
    lastBB = BB;
    vexDelay(10);
  }
}

int main() {
  vexDelay(200);
  task GP1(positioning);
  task GP2(GPSpositioning);
  task GP(posConfig);
  task BS(base);
  task FW(flywheelContorl);
  vexDelay(200);
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  while (true) {
    vexDelay(100);
  }
}
