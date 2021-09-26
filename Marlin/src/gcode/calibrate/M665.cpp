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

#if IS_KINEMATIC

#include "../gcode.h"
#include "../../module/motion.h"

#if ENABLED(DELTA)

  #include "../../module/delta.h"
  /**
   * M665: Set delta configurations
   *
   *    H = delta height
   *    L = diagonal rod
   *    R = delta radius
   *    S = segments per second
   *    X = Alpha (Tower 1) angle trim
   *    Y = Beta  (Tower 2) angle trim
   *    Z = Gamma (Tower 3) angle trim
   *    A = Alpha (Tower 1) diagonal rod trim
   *    B = Beta  (Tower 2) diagonal rod trim
   *    C = Gamma (Tower 3) diagonal rod trim
   */
  void GcodeSuite::M665() {
    if (parser.seenval('H')) delta_height              = parser.value_linear_units();
    if (parser.seenval('L')) delta_diagonal_rod        = parser.value_linear_units();
    if (parser.seenval('R')) delta_radius              = parser.value_linear_units();
    if (parser.seenval('S')) segments_per_second       = parser.value_float();
    if (parser.seenval('X')) delta_tower_angle_trim.a  = parser.value_float();
    if (parser.seenval('Y')) delta_tower_angle_trim.b  = parser.value_float();
    if (parser.seenval('Z')) delta_tower_angle_trim.c  = parser.value_float();
    if (parser.seenval('A')) delta_diagonal_rod_trim.a = parser.value_float();
    if (parser.seenval('B')) delta_diagonal_rod_trim.b = parser.value_float();
    if (parser.seenval('C')) delta_diagonal_rod_trim.c = parser.value_float();
    recalc_delta_settings();
  }

#elif IS_SCARA

  #include "../../module/scara.h"

  /**
   * M665: Set SCARA settings
   *
   * Parameters:
   *
   *   S[segments-per-second] - Segments-per-second
   *   P[theta-psi-offset]    - Theta-Psi offset, added to the shoulder (A/X) angle
   *   T[theta-offset]        - Theta     offset, added to the elbow    (B/Y) angle
   *   Z[z-offset]            - Z offset, added to Z
   *
   *   A, P, and X are all aliases for the shoulder angle
   *   B, T, and Y are all aliases for the elbow angle
   */
  void GcodeSuite::M665() {
    if (parser.seenval('S')) segments_per_second = parser.value_float();

    #if HAS_SCARA_OFFSET

      if (parser.seenval('Z')) scara_home_offset.z = parser.value_linear_units();

      if (parser.seenval('X')) scara_tower_offset.y = parser.value_linear_units();

      if (parser.seenval('Y')) scara_tower_offset.y = parser.value_linear_units();

      const bool hasA = parser.seenval('A'), hasP = parser.seenval('P');
      const uint8_t sumAP = hasA + hasP;
      if (sumAP) {
        if (sumAP == 1)
          scara_home_offset.a = parser.value_float();
        else {
          SERIAL_ERROR_MSG("Only one of A or P is allowed.");
          return;
        }
      }

      const bool hasB = parser.seenval('B'), hasT = parser.seenval('T');
      const uint8_t sumBT = hasB + hasT;
      if (sumBT) {
        if (sumBT == 1)
          scara_home_offset.b = parser.value_float();
        else {
          SERIAL_ERROR_MSG("Only one of B or T is allowed.");
          return;
        }
      }

    #endif // HAS_SCARA_OFFSET
  }

#endif

#endif // IS_KINEMATIC
