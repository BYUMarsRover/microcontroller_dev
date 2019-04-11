#include "PD_SM.h"
#ifndef __VEL_SM__
#define __VEL_SM__
static enum VEL_state{VEL_STOP, VEL_CCW, VEL_CW} velState;

#define STOP 0
#define CCW 1
#define CW 2
#define SPEED 50

byte direction;

//init function
void VEL_init() {
  velState = VEL_STOP;
  direction = STOP;
}

//the tick function for the sm
void VEL_tick() {
  switch (velState) {
    case VEL_STOP:
      if (direction = CW) {
        velState = VEL_CW;
        turret_cw(SPEED);
      }
      else if (direction == CCW) {
        velState = VEL_CCW;
        turret_ccw(SPEED);
      }
    break;

    case VEL_CCW:
      if (direction == STOP) {
        stop_turret();
        velState = VEL_STOP;
      }
    break;

    case VEL_CW:
      if (direction == STOP) {
        stop_turret();
        velState = VEL_STOP;
      }
    break;
  }
}
#endif
