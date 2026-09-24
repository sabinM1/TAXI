/*
 * Motor_30 — hold both motors at 30% forward speed.
 * Uses the xmotionV3 library bundled alongside this sketch.
 *
 * xmotion.Forward(speed, time) takes speed as 0-100 percent and maps
 * it to PWM internally, so 30 == true 30% duty cycle.
 */

#include "xmotionV3.h"

#define MOTOR_SPEED_PCT 30

void setup()
{
  xmotion.SETUP();
  xmotion.Forward(MOTOR_SPEED_PCT, 0);  // time 0 = no blocking delay
}

void loop()
{
  // Nothing to do — PWM keeps running on its own.
}
