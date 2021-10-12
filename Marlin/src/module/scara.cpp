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

/**
 * scara.cpp
 */

#include "../inc/MarlinConfig.h"

#if IS_SCARA

#include "scara.h"
#include "motion.h"
#include "planner.h"
#include "endstops.h"
#include "../MarlinCore.h"

#define DEBUG_OUT ENABLED(DEBUG_SCARA_KINEMATICS)
#include "../core/debug_out.h"


float segments_per_second = TERN(AXEL_TPARA, TPARA_SEGMENTS_PER_SECOND, SCARA_SEGMENTS_PER_SECOND);
//scara_offset.set(SCARA_OFFSET_X , SCARA_OFFSET_Y );
//scara_home_offset.set( PHI_HOME_POS , THETA_HOME_POS , Z_HOME_POS );

#if EITHER(MORGAN_SCARA, MP_SCARA)

//xy_pos_t scara_offset = { SCARA_OFFSET_X , SCARA_OFFSET_Y };
  // MP_SCARA uses arm angles for AB home position //@Ofir-T: maybe move it to sanity checks or equivalent?
      //#ifndef POLAR_HOME_POS
      //  #define POLAR_HOME_POS  { 0 , 0 } // degrees
      //#endif

  /**
   * Morgan SCARA Forward Kinematics. Results in 'cartes'.
   * Maths and first version by QHARLEY.
   * Integrated into Marlin and slightly restructured by Joachim Cerny.
   */
  void forward_kinematics(const_float_t phi, const_float_t theta) {
    const float phi_sin = sin(RADIANS(phi)) * L1,
                phi_cos = cos(RADIANS(phi)) * L1,
                psi_sin = sin(RADIANS(SUM_TERN(MP_SCARA, theta, phi))) * L2, //@OfirT: b_sin = sin(RADIANS(SUM_TERN(MP_SCARA, b, a))) * L2,
                psi_cos = cos(RADIANS(SUM_TERN(MP_SCARA, theta, phi))) * L2; //@OfirT: b_cos = cos(RADIANS(SUM_TERN(MP_SCARA, b, a))) * L2;

    cartes.x = phi_cos + psi_cos + scara_offset.x;  
    cartes.y = phi_sin + psi_sin + scara_offset.y;  

    
      DEBUG_ECHOLNPGM(
        "SCARA FK:\n  phi=", phi,
        "\n  theta=", theta,
        "\n  psi=", phi + theta,
        "\n  phi_sin=", phi_sin,
        "\n  phi_cos=", phi_cos,
        "\n  theta_sin=", psi_sin,
        "\n  theta_cos=", psi_cos
      );
      DEBUG_ECHOLNPGM("\n  cartes (X,Y) = (", cartes.x,", ", cartes.y, ")");
    //*/
  }

#endif

#if ENABLED(MORGAN_SCARA)

  void scara_set_axis_is_at_home(const AxisEnum axis) {
    if (axis == Z_AXIS)
      current_position.z = Z_HOME_POS;
    else {
      // MORGAN_SCARA uses a Cartesian XY home position
      xyz_pos_t homeposition = { X_HOME_POS, Y_HOME_POS, Z_HOME_POS };
      //DEBUG_ECHOLNPGM_P(PSTR("homeposition X"), homeposition.x, SP_Y_LBL, homeposition.y);

      delta = homeposition;
      forward_kinematics(delta.a, delta.b);
      current_position[axis] = cartes[axis];

      //DEBUG_ECHOLNPGM_P(PSTR("Cartesian X"), current_position.x, SP_Y_LBL, current_position.y);
      update_software_endstops(axis);
    }
  }

  /**
   * Morgan SCARA Inverse Kinematics. Results are stored in 'delta'.
   *
   * See https://reprap.org/forum/read.php?185,283327
   *
   * Maths and first version by QHARLEY.
   * Integrated into Marlin and slightly restructured by Joachim Cerny.
   */
  void inverse_kinematics(const xyz_pos_t &raw) {
    float C2, S2, SK1, SK2, THETA, PSI;

    // Translate SCARA to standard XY with scaling factor
    const xy_pos_t spos = raw - scara_offset;

    const float H2 = HYPOT2(spos.x, spos.y);
    if (L1 == L2)
      C2 = H2 / L1_2_2 - 1;
    else
      C2 = (H2 - (L1_2 + L2_2)) / (2.0f * L1 * L2);

    LIMIT(C2, -1, 1);

    S2 = SQRT(1.0f - sq(C2));

    // Unrotated Arm1 plus rotated Arm2 gives the distance from Center to End
    SK1 = L1 + L2 * C2;

    // Rotated Arm2 gives the distance from Arm1 to Arm2
    SK2 = L2 * S2;

    // Angle of Arm1 is the difference between Center-to-End angle and the Center-to-Elbow
    THETA = ATAN2(SK1, SK2) - ATAN2(spos.x, spos.y);

    // Angle of Arm2
    PSI = ATAN2(S2, C2);

    delta.set(DEGREES(THETA), DEGREES(SUM_TERN(MORGAN_SCARA, PSI, THETA)), raw.z);

    /*
      DEBUG_POS("SCARA IK", raw);
      DEBUG_POS("SCARA IK", delta);
      DEBUG_ECHOLNPGM("  SCARA (x,y) ", sx, ",", sy, " C2=", C2, " S2=", S2, " Theta=", THETA, " Psi=", PSI);
    //*/
  }

#elif ENABLED(MP_SCARA)

  // Homes either Z_AXIS, or XY together 
  void scara_set_axis_is_at_home(const AxisEnum axis) {
    if (axis == Z_AXIS)
      current_position.z = scara_home_offset.z;
    else {
      //ab_float_t scara_home_offset = { PHI_HOME_POS , THETA_HOME_POS };
      DEBUG_ECHOLNPGM("Homeposition A:", scara_home_offset.a, " B:", scara_home_offset.b, " Z:", Z_HOME_POS);
      
      delta = scara_home_offset;
      //inverse_kinematics(scara_home_offset); //@OfirT: what is this used for? maybe syncing x,y to p,t i cancelled it with no effect
      forward_kinematics(delta.a, delta.b);
      current_position[axis] = cartes[axis];

      DEBUG_ECHOLNPGM_P(PSTR("Cartesian X"), current_position.x, SP_Y_LBL, current_position.y);
      update_software_endstops(axis);
    }
  }

  //@OfirT
  void inverse_kinematics(const xyz_pos_t &raw) { // angles are in radians
    const float x = raw.x, y = raw.y, c = HYPOT(x, y), quad_flipper = 1,//quad_flipper = (y < 0 && x > 0 ? -1 : 1),  // IN CONSTRUCTION- flips handednes in opposite quadrant (righty:4, lefty:3)
                ARG_XY = ((x<0 && y<0) ?  ATAN2(y, x)+2*M_PI : ATAN2(y, x)),                                        // condition for arg(x,y) in quadrant 3
                PHI = ARG_XY + (ACOS((sq(c) + sq(L1) - sq(L2)) / (2.0f * c * L1)) * SCARA_HANDEDNESS * quad_flipper), // make sure it's staying efficient enough to operate smoothly
                PSI = ARG_XY - (ACOS((sq(c) + sq(L2) - sq(L1)) / (2.0f * c * L2)) * SCARA_HANDEDNESS * quad_flipper); 

    delta.set(DEGREES(PHI), DEGREES(PSI), raw.z);
    
      DEBUG_POS(" SCARA IK", raw);
      DEBUG_POS(" SCARA IK", delta);
      SERIAL_ECHOLNPGM("   SCARA (x,y) ", x, ",", y,
                        "\n   Phi=", DEGREES(PHI), "(", PHI, ")"
                        "\n   Theta=", DEGREES(PSI-PHI), "(", PSI-PHI, ")"
                        "\n   Psi=", DEGREES(PSI), "(", PSI, ")"
                        "\n   ARG_XY=", DEGREES(ARG_XY), "(", ARG_XY, ")"
        );
    //*/
  }

#elif ENABLED(AXEL_TPARA)

  static constexpr xyz_pos_t robot_offset = { TPARA_OFFSET_X, TPARA_OFFSET_Y, TPARA_OFFSET_Z };

  void scara_set_axis_is_at_home(const AxisEnum axis) {
    if (axis == Z_AXIS)
      current_position.z = Z_HOME_POS;
    else {
      xyz_pos_t homeposition = { X_HOME_POS, Y_HOME_POS, Z_HOME_POS };
      //DEBUG_ECHOLNPGM_P(PSTR("homeposition X"), homeposition.x, SP_Y_LBL, homeposition.y, SP_Z_LBL, homeposition.z);

      inverse_kinematics(homeposition);
      forward_kinematics(delta.a, delta.b, delta.c);
      current_position[axis] = cartes[axis];

      //DEBUG_ECHOLNPGM_P(PSTR("Cartesian X"), current_position.x, SP_Y_LBL, current_position.y);
      update_software_endstops(axis);
    }
  }

  // Convert ABC inputs in degrees to XYZ outputs in mm
  void forward_kinematics(const_float_t a, const_float_t b, const_float_t c) {
    const float w = c - b,
                r = L1 * cos(RADIANS(b)) + L2 * sin(RADIANS(w - (90 - b))),
                x = r  * cos(RADIANS(a)),
                y = r  * sin(RADIANS(a)),
                rho2 = L1_2 + L2_2 - 2.0f * L1 * L2 * cos(RADIANS(w));

    cartes = robot_offset + xyz_pos_t({ x, y, SQRT(rho2 - sq(x) - sq(y)) });
  }

  // Home YZ together, then X (or all at once). Based on quick_home_xy & home_delta
  void home_TPARA() {
    // Init the current position of all carriages to 0,0,0
    current_position.reset();
    destination.reset();
    sync_plan_position();

    // Disable stealthChop if used. Enable diag1 pin on driver.
    #if ENABLED(SENSORLESS_HOMING)
      TERN_(X_SENSORLESS, sensorless_t stealth_states_x = start_sensorless_homing_per_axis(X_AXIS));
      TERN_(Y_SENSORLESS, sensorless_t stealth_states_y = start_sensorless_homing_per_axis(Y_AXIS));
      TERN_(Z_SENSORLESS, sensorless_t stealth_states_z = start_sensorless_homing_per_axis(Z_AXIS));
    #endif

    //const int x_axis_home_dir = TOOL_X_HOME_DIR(active_extruder);

    //const xy_pos_t pos { max_length(X_AXIS) , max_length(Y_AXIS) };
    //const float mlz = max_length(X_AXIS),

    // Move all carriages together linearly until an endstop is hit.
    //do_blocking_move_to_xy_z(pos, mlz, homing_feedrate(Z_AXIS));

    current_position.x = 0 ;
    current_position.y = 0 ;
    current_position.z = max_length(Z_AXIS) ;
    line_to_current_position(homing_feedrate(Z_AXIS));
    planner.synchronize();

    // Re-enable stealthChop if used. Disable diag1 pin on driver.
    #if ENABLED(SENSORLESS_HOMING)
      TERN_(X_SENSORLESS, end_sensorless_homing_per_axis(X_AXIS, stealth_states_x));
      TERN_(Y_SENSORLESS, end_sensorless_homing_per_axis(Y_AXIS, stealth_states_y));
      TERN_(Z_SENSORLESS, end_sensorless_homing_per_axis(Z_AXIS, stealth_states_z));
    #endif

    endstops.validate_homing_move();

    // At least one motor has reached its endstop.
    // Now re-home each motor separately.
    homeaxis(A_AXIS);
    homeaxis(C_AXIS);
    homeaxis(B_AXIS);

    // Set all carriages to their home positions
    // Do this here all at once for Delta, because
    // XYZ isn't ABC. Applying this per-tower would
    // give the impression that they are the same.
    LOOP_LINEAR_AXES(i) set_axis_is_at_home((AxisEnum)i);

    sync_plan_position();
  }

  void inverse_kinematics(const xyz_pos_t &raw) {
    const xyz_pos_t spos = raw - robot_offset;

    const float RXY = SQRT(HYPOT2(spos.x, spos.y)),
                RHO2 = NORMSQ(spos.x, spos.y, spos.z),
                //RHO = SQRT(RHO2),
                LSS = L1_2 + L2_2,
                LM = 2.0f * L1 * L2,

                CG = (LSS - RHO2) / LM,
                SG = SQRT(1 - POW(CG, 2)), // Method 2
                K1 = L1 - L2 * CG,
                K2 = L2 * SG,

                // Angle of Body Joint
                THETA = ATAN2(spos.y, spos.x),

                // Angle of Elbow Joint
                //GAMMA = ACOS(CG),
                GAMMA = ATAN2(SG, CG), // Method 2

                // Angle of Shoulder Joint, elevation angle measured from horizontal (r+)
                //PHI = asin(spos.z/RHO) + asin(L2 * sin(GAMMA) / RHO),
                PHI = ATAN2(spos.z, RXY) + ATAN2(K2, K1),   // Method 2

                // Elbow motor angle measured from horizontal, same frame as shoulder  (r+)
                PSI = PHI + GAMMA;

    delta.set(DEGREES(THETA), DEGREES(PHI), DEGREES(PSI));

    //SERIAL_ECHOLNPGM(" SCARA (x,y,z) ", spos.x , ",", spos.y, ",", spos.z, " Rho=", RHO, " Rho2=", RHO2, " Theta=", THETA, " Phi=", PHI, " Psi=", PSI, " Gamma=", GAMMA);
  }

#endif

void scara_report_positions() {
  SERIAL_ECHOLNPGM("SCARA Phi:", planner.get_axis_position_degrees(A_AXIS)
    #if ENABLED(AXEL_TPARA)
      , "  Phi:", planner.get_axis_position_degrees(B_AXIS)
      , "  Psi:", planner.get_axis_position_degrees(C_AXIS)
    #else
      , "  Psi(Phi+Theta)" TERN_(MORGAN_SCARA, "+Theta") ":", planner.get_axis_position_degrees(B_AXIS)
    #endif
  );
  SERIAL_EOL();
}

#endif // IS_SCARA
