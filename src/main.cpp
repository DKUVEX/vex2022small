#include "tasks.h"

using namespace vex;
using namespace std;

void autonomous(void) {
  //runAuton();
  //oneminute310p();
}

void usercontrol(void) {
  bool lastL1 = false;
  
  sol.set(0);
  while (1) {

    if(BB) fwState = fw_OFF;
    else if(L2 && R1) fwState = fw_HSPD;
    else if(L2 && R2) fwState = fw_LSPD;

    if(fwState == fw_LSPD && kCount > 0) {
      kick(1);
      kCount -= 1;
    }
    else if(fwState == fw_HSPD && ifSpeedOK && L1 && !lastL1){
      kick(1);
    }

    intake(100*(R2-R1)*!L2);
    //index(100*BA);
    
    lastL1 = L1;
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
  task LA(launch);

  vexDelay(200);
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  while (true) {
    vexDelay(100);
  }
}
