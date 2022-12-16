#include "tasks.h"

using namespace vex;
using namespace std;

void autonomous(void) {
  chState = ctrl_AUTONOMOUS;
  
  // LA.spin(fwd, 20, voltageUnits::mV);
  // LB.spin(fwd, 20, voltageUnits::mV);
  // if (gps_xpos == 20 && gps_ypos == 20){
  //   break;
  // }
  //runAuton();
  //oneminute310p();
  //测试直到小车一秒走1块 左转一秒180度 右转1秒180度
  //第一次前进长度还没确定
  intake(-100);
  fwState = fw_HSPD;

  mov_fwd(1400);
  delay(1500);
  rotate_left(900);
  delay(100);
  kick(1);
  delay(500);
  kick(1);
  delay(500);
  kick(1);
  delay(500);
  rotate_right(680);
  mov_fwd(1814);
  delay(2500);
  rotate_left(650);
  delay(100);
  kick(1);
  delay(500);
  kick(1);
  delay(500);
  kick(1);
  delay(500);
  // kick(3);
  rotate_right(150);
  mov_fwd(553);
  rotate_left(300);
  mov_fwd(453);
  // rotate_right(80);
  mov_fwd(2300);
  intake(0);
  fwState = fw_OFF;  
  // mov_bwd(500);
  
  // mov_fwd(1000);
  // rotate_left(1000);
  // rotate_right(1000);
  
  // drift(50, 0, 1, 0.8);
  // drift(-100, 0, 1, 0.8);
}

void usercontrol(void) {
  bool lastL1 = false;
  chState = ctrl_DEFAULT;
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

    intake(120*(R2-R1)*!L2);
    //index(100*BA);
    roller(100*BY);
    extend(-20*UP);
    lastL1 = L1;
    // cout<<Hor.rotation(deg)<<"  "<<Ver.rotation(deg)<<endl;
    //printScreen(10,140,"x:%.2f y:%.2f v4gyro:%.2f v5gyro:%.2f",omniPos[0], omniPos[1], -v4gyro.rotation(),-Gyro.rotation());

    vexDelay(10);
  }
}

int main() {
  vexDelay(200);
  // task GP1(positioning);
  task GP2(GPSpositioning);
  task GP3(Inertialposiyioning);
  task GP4(Filtpositioning);
  // task GP(posConfig);
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
