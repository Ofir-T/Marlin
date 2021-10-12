/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2020 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 *
 * Based on Sprinter and grbl.
 * Copyright (c) 2011 Camiel Gubbels / Erik van der Zalm
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 */

#include "../../inc/MarlinConfig.h"

#define DEBUG_OUT ENABLED(DEBUG_SCARA_KINEMATICS)
#include "../../core/debug_out.h"

#if ENABLED(MORGAN_SCARA)

#include "../gcode.h"
#include "../../module/scara.h"
#include "../../module/motion.h"
#include "../../MarlinCore.h" // for IsRunning()

inline bool SCARA_move_to_cal(const uint8_t delta_a, const uint8_t delta_b) {
  if (IsRunning()) {
    forward_kinematics(delta_a, delta_b);
    do_blocking_move_to_xy(cartes);
    return true;
  }
  return false;
}

/**
 * M360: SCARA calibration: Move to cal-position ThetaA (0 deg calibration)
 */
bool GcodeSuite::M360() {
  SERIAL_ECHOLNPGM(" Cal: Theta 0");
  return SCARA_move_to_cal(0, 120);
}

/**
 * M361: SCARA calibration: Move to cal-position ThetaB (90 deg calibration - steps per degree)
 */
bool GcodeSuite::M361() {
  SERIAL_ECHOLNPGM(" Cal: Theta 90");
  return SCARA_move_to_cal(90, 130);
}

/**
 * M362: SCARA calibration: Move to cal-position PsiA (0 deg calibration)
 */
bool GcodeSuite::M362() {
  SERIAL_ECHOLNPGM(" Cal: Psi 0");
  return SCARA_move_to_cal(60, 180);
}

/**
 * M363: SCARA calibration: Move to cal-position PsiB (90 deg calibration - steps per degree)
 */
bool GcodeSuite::M363() {
  SERIAL_ECHOLNPGM(" Cal: Psi 90");
  return SCARA_move_to_cal(50, 90);
}

/**
 * M364: SCARA calibration: Move to cal-position PsiC (90 deg to Theta calibration position)
 */
bool GcodeSuite::M364() {
  SERIAL_ECHOLNPGM(" Cal: Theta-Psi 90");
  return SCARA_move_to_cal(45, 135);
}
#endif //MORGAN_SCARA

#if ENABLED(MP_SCARA)

#include "../gcode.h"
#include "../../module/scara.h"
#include "../../module/motion.h"
#include "../../MarlinCore.h" // for IsRunning()

inline bool SCARA_move_to_cal(const uint8_t delta_a, const uint8_t delta_b, const uint8_t fr_mm_s = 20) {
  if (IsRunning()) {
    if(!position_is_reachable_degrees(delta_a,delta_b)) {
      SERIAL_ECHOLNPGM("The requested position is out the machine bounds! Double check your linkage sizes");
      return false;
    }
    forward_kinematics(delta_a, delta_b);
    do_blocking_move_to_xy(cartes, fr_mm_s);
    return true;
  }
  SERIAL_ECHOLNPGM("The machine is not running");
  return false;
}

/**
 * M360: SCARA calibration: Move to cal-position ThetaA (0 deg calibration)
 */
void GcodeSuite::M360() {
  SERIAL_ECHOLNPGM(" Cal: (P,T) = (0,0)");
  if (!SCARA_move_to_cal(0, 0)) return;
  scara_report_positions();
}

/**
 * M361: SCARA calibration: Move to cal-position ThetaB (90 deg calibration - steps per degree)
 */
void GcodeSuite::M361() {
  SERIAL_ECHOLNPGM(" Cal: (P,T) = (90,0)");
  if (!SCARA_move_to_cal(90, 0)) return;
  scara_report_positions();
}

/**
 * M362: SCARA calibration: Move to cal-position PsiA (0 deg calibration)
 */
void GcodeSuite::M362() {
  SERIAL_ECHOLNPGM(" Cal: (P,T) = (0,90)");
  if (!SCARA_move_to_cal(0, 90)) return;
  scara_report_positions();
}

/**
 * M363: SCARA calibration: Move to cal-position PsiB (90 deg calibration - steps per degree)
 */
void GcodeSuite::M363() {
  SERIAL_ECHOLNPGM(" Cal: (P,T) = (180,0)");
  if (!SCARA_move_to_cal(180, 0)) return;
  scara_report_positions();
}

/**
 * M364: SCARA calibration: Move to cal-position PsiC (90 deg to Theta calibration position)
 */
void GcodeSuite::M364() {
  SERIAL_ECHOLNPGM(" Cal: (P,T) = (180,90)");
  if (!SCARA_move_to_cal(180, 90)) return;
  scara_report_positions();
}
#endif //MP_SCARA