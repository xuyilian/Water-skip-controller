/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2011-2022 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * power_distribution_quadrotor.c - Crazyflie stock power distribution code
 */


#include "power_distribution.h"

#include <string.h>
#include "debug.h"
#include "log.h"
#include "param.h"
#include "num.h"
#include "autoconf.h"
#include "config.h"
#include "math.h"
#include "platform_defaults.h"

#if (!defined(CONFIG_MOTORS_REQUIRE_ARMING) || (CONFIG_MOTORS_REQUIRE_ARMING == 0)) && defined(CONFIG_MOTORS_DEFAULT_IDLE_THRUST) && (CONFIG_MOTORS_DEFAULT_IDLE_THRUST > 0)
    #error "CONFIG_MOTORS_REQUIRE_ARMING must be defined and not set to 0 if CONFIG_MOTORS_DEFAULT_IDLE_THRUST is greater than 0"
#endif
#ifndef CONFIG_MOTORS_DEFAULT_IDLE_THRUST
#  define DEFAULT_IDLE_THRUST 0
#else
#  define DEFAULT_IDLE_THRUST CONFIG_MOTORS_DEFAULT_IDLE_THRUST
#endif

static uint32_t idleThrust = DEFAULT_IDLE_THRUST;

// ---------------- tangential-spin mixer (water-hop board, e.g. "cf21blspin") ----------------
// CF2.1 Brushless board, 4x RCinPOWER GTS V3 0802 22000KV. Geometry (confirmed
// with the team via cfclient's motor-power-set-index test, 2026-07-23 --
// motor-to-role mapping AND corner positions below are bench-verified, not
// inferred):
//   - Motors 2 & 4: one diagonal, 45 deg LEFT-of-front, mounted HORIZONTAL,
//     thrusting tangentially/backwards -- pure yaw-spin, zero lift authority.
//   - Motors 1 & 3: the perpendicular diagonal, 45 deg RIGHT-of-front, mounted
//     VERTICAL/upward -- the lift pair, both spinning the SAME direction (both
//     contribute CCW reaction torque), so they have no differential yaw
//     authority of their own. All yaw comes from motors 2/4's tangential
//     thrust instead, which is why this mixer carries no control->yaw term.
//     Confirmed corner positions, top-down, front-of-drone up: m1 = front-
//     right, m3 = back-left -- i.e. the SAME corners as M1/M3 in the standard
//     Crazyflie X-mode layout (M1 & M4 = front, per crtp_commander_rpyt.c;
//     roll sign in powerDistributionLegacy below then places M1 on the right).
//     A motor's roll/pitch torque per unit thrust depends only on its own
//     position, not on what the other two motors do, so m1/m3 below reuse the
//     legacy mixer's exact M1/M3 coefficients unchanged.
//   - Motors 2/4 run at the fixed, independent spinMotorThrust regardless of
//     control->thrust -- this is what keeps the whole-drone yaw rate up
//     through DROP even as lift thrust falls to near zero, and lets spin rate
//     be pushed higher independent of lift authority.
//
// Not compile-time selected: mixerType is a runtime param (like idleThrust) so
// the SAME firmware image serves the legacy 4-motor boards (cf2, cf2bl) and
// this board -- Python sets it once at connect, keyed off MACHINES. It
// defaults to 0 (legacy mixer) and is NOT persistent, so a board that boots
// without Python setting it stays on the safe, well-tested legacy mixer
// rather than silently inheriting a stale value from a previous session.
//
// BENCH-VERIFY BEFORE FLYING -- motor-to-role mapping and corner positions are
// now confirmed and the m1/m3 coefficients below are derived (not guessed)
// from the standard CF corner map, but this has NOT yet been confirmed with
// motors actually spinning. With props at low idle, command a small pitch
// stick input, then a small roll stick input, and confirm motors 1/3 differ
// (one speeds up, one slows down) in the CORRECT direction for each -- both
// inputs should produce a visible, correctly-signed differential, not just
// one of them. If either is backwards, the most likely cause is the lift
// pair's mounting being rotated 180 deg from the assumed front-right/back-
// left orientation -- swap which of m1/m3 gets which sign below.
static uint8_t mixerType = 0;          // 0 = legacy 4-rotor, 1 = tangential-spin
static uint32_t spinMotorThrust = 0;   // fixed thrust for motors 2 & 4 (tangential/yaw)

// Per-motor thrust ceiling applied in powerDistributionCap(), in addition to
// the legacy differential-preserving reduction. Defaults to UINT16_MAX (a
// no-op) so existing boards are unaffected unless Python sets it lower (e.g.
// 60000 for a motor with a lower rated max).
static uint32_t motorThrustCeiling = UINT16_MAX;

static void powerDistributionTangentialSpin(const control_t *control, motors_thrust_uncapped_t* motorThrustUncapped)
{
  int16_t r = control->roll / 2.0f;
  int16_t p = control->pitch / 2.0f;

  motorThrustUncapped->motors.m2 = spinMotorThrust;
  motorThrustUncapped->motors.m4 = spinMotorThrust;

  // m1 (front-right) / m3 (back-left) sit at the SAME physical corners as the
  // legacy mixer's M1/M3 (see powerDistributionLegacy below and
  // crtp_commander_rpyt.c's "M1 & M4 are defined as front" for the standard
  // CF corner map) -- a motor's roll/pitch torque per unit thrust depends
  // only on its own position, not on what the other two motors do, so these
  // are the legacy M1/M3 coefficients reused unchanged (no yaw term: yaw
  // authority comes entirely from m2/m4's tangential thrust).
  motorThrustUncapped->motors.m1 = control->thrust - r + p;
  motorThrustUncapped->motors.m3 = control->thrust + r - p;
}

int powerDistributionMotorType(uint32_t id)
{
  return 1;
}

uint16_t powerDistributionStopRatio(uint32_t id)
{
  return 0;
}

void powerDistributionInit(void)
{
  #if (!defined(CONFIG_MOTORS_REQUIRE_ARMING) || (CONFIG_MOTORS_REQUIRE_ARMING == 0))
  if(idleThrust > 0) {
    DEBUG_PRINT("WARNING: idle thrust will be overridden with value 0. Autoarming can not be on while idle thrust is higher than 0. If you want to use idle thust please use use arming\n");
  }
  #endif
}

bool powerDistributionTest(void)
{
  bool pass = true;
  return pass;
}

static uint16_t capMinThrust(float thrust, uint32_t minThrust) {
  if (thrust < minThrust) {
    return minThrust;
  }

  return thrust;
}

static void powerDistributionLegacy(const control_t *control, motors_thrust_uncapped_t* motorThrustUncapped)
{
  int16_t r = control->roll / 2.0f;
  int16_t p = control->pitch / 2.0f;

  motorThrustUncapped->motors.m1 = control->thrust - r + p + control->yaw;
  motorThrustUncapped->motors.m2 = control->thrust - r - p - control->yaw;
  motorThrustUncapped->motors.m3 = control->thrust + r - p + control->yaw;
  motorThrustUncapped->motors.m4 = control->thrust + r + p - control->yaw;
}

static void powerDistributionForceTorque(const control_t *control, motors_thrust_uncapped_t* motorThrustUncapped) {
  static float motorForces[STABILIZER_NR_OF_MOTORS];

  const float arm = 0.707106781f * ARM_LENGTH;
  const float rollPart = 0.25f / arm * control->torqueX;
  const float pitchPart = 0.25f / arm * control->torqueY;
  const float thrustPart = 0.25f * control->thrustSi; // N (per rotor)
  const float yawPart = 0.25f * control->torqueZ / THRUST2TORQUE;

  motorForces[0] = thrustPart - rollPart - pitchPart - yawPart;
  motorForces[1] = thrustPart - rollPart + pitchPart + yawPart;
  motorForces[2] = thrustPart + rollPart + pitchPart - yawPart;
  motorForces[3] = thrustPart + rollPart - pitchPart + yawPart;

  for (int motorIndex = 0; motorIndex < STABILIZER_NR_OF_MOTORS; motorIndex++) {
    float motorForce = motorForces[motorIndex];
    if (motorForce < 0.0f) {
      motorForce = 0.0f;
    }

    motorThrustUncapped->list[motorIndex] = motorForce / THRUST_MAX * UINT16_MAX;
  }
}

/**
 * @brief Allows for direct control of motor power with clipping
 *
 * This function applies clipping to the motor values, which is different to
 * the "capping" behaviour found in powerDistributionForceTorque() - which
 * instead prioritizes stability rather than thrust.
 */
static void powerDistributionForce(const control_t *control, motors_thrust_uncapped_t* motorThrustUncapped) {
  for (int i = 0; i < STABILIZER_NR_OF_MOTORS; i++) {
    float f = control->normalizedForces[i];

    if (f < 0.0f) {
      f = 0.0f;
    }

    if (f > 1.0f) {
      f = 1.0f;
    }

    motorThrustUncapped->list[i] = f * UINT16_MAX;
  }
}

void powerDistribution(const control_t *control, motors_thrust_uncapped_t* motorThrustUncapped)
{
  switch (control->controlMode) {
    case controlModeLegacy:
      if (mixerType == 1) {
        powerDistributionTangentialSpin(control, motorThrustUncapped);
      } else {
        powerDistributionLegacy(control, motorThrustUncapped);
      }
      break;
    case controlModeForceTorque:
      powerDistributionForceTorque(control, motorThrustUncapped);
      break;
    case controlModeForce:
      powerDistributionForce(control, motorThrustUncapped);
      break;
    default:
      // Nothing here
      break;
  }
}

bool powerDistributionCap(const motors_thrust_uncapped_t* motorThrustBatCompUncapped, motors_thrust_pwm_t* motorPwm)
{
  const int32_t maxAllowedThrust = motorThrustCeiling;
  bool isCapped = false;

  // Find highest thrust
  int32_t highestThrustFound = 0;
  for (int motorIndex = 0; motorIndex < STABILIZER_NR_OF_MOTORS; motorIndex++)
  {
    const int32_t thrust = motorThrustBatCompUncapped->list[motorIndex];
    if (thrust > highestThrustFound)
    {
      highestThrustFound = thrust;
    }
  }

  int32_t reduction = 0;
  if (highestThrustFound > maxAllowedThrust)
  {
    reduction = highestThrustFound - maxAllowedThrust;
    isCapped = true;
  }

  for (int motorIndex = 0; motorIndex < STABILIZER_NR_OF_MOTORS; motorIndex++)
  {
    int32_t thrustCappedUpper = motorThrustBatCompUncapped->list[motorIndex] - reduction;
    motorPwm->list[motorIndex] = capMinThrust(thrustCappedUpper, powerDistributionGetIdleThrust());
  }

  return isCapped;
}

uint32_t powerDistributionGetIdleThrust()
{
  int32_t thrust = idleThrust;
  #if (!defined(CONFIG_MOTORS_REQUIRE_ARMING) || (CONFIG_MOTORS_REQUIRE_ARMING == 0))
    thrust = 0;
  #endif
  return thrust;
}

float powerDistributionGetMaxThrust() {
  return STABILIZER_NR_OF_MOTORS * THRUST_MAX;
}

/**
 * Power distribution parameters
 */
PARAM_GROUP_START(powerDist)
/**
 * @brief Motor thrust to set at idle (default: 0)
 *
 * This is often needed for brushless motors as
 * it takes time to start up the motor. Then a
 * common value is between 3000 - 6000.
 */
PARAM_ADD_CORE(PARAM_UINT32 | PARAM_PERSISTENT, idleThrust, &idleThrust)
/**
 * @brief Motor mixer to use (default: 0)
 *
 * 0 = legacy 4-rotor mix (cf2, cf2bl). 1 = tangential-spin mix for the
 * water-hop board: motors 2 & 4 are fixed-thrust tangential yaw spinners,
 * motors 1 & 3 are the vertical lift pair. NOT persistent: resets to the
 * safe legacy mixer (0) on every boot until explicitly set, so a session
 * that never sets this can't inherit a stale value from a previous board.
 */
PARAM_ADD_CORE(PARAM_UINT8, mixerType, &mixerType)
/**
 * @brief Fixed thrust for motors 2 & 4 in the tangential-spin mixer (default: 0)
 *
 * Only used when mixerType == 1. Independent of the roll/pitch/thrust
 * setpoint -- this is what keeps the vehicle spinning through DROP/hop
 * regardless of how low the lift-motor thrust is commanded.
 */
PARAM_ADD_CORE(PARAM_UINT32, spinThrust, &spinMotorThrust)
/**
 * @brief Per-motor thrust ceiling (default: 65535, i.e. no cap)
 *
 * Applied to ALL motors/mixers alongside the existing idle-thrust floor.
 * Set lower (e.g. 60000) for a motor with a lower rated maximum.
 */
PARAM_ADD_CORE(PARAM_UINT32, thrustCap, &motorThrustCeiling)
PARAM_GROUP_STOP(powerDist)
