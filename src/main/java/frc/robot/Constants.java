// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  // Operator Constants
  public static class OperatorConstants {
    // Joystick port
    public static final int kDriverControllerPort = 0;
  }


   public static class ArmSubsystemConstants {
    // Direction enum for arm pivot movement
    public enum DIRECTION {
      FORWARD,
      REVERSE
    }
    // Arm Subsystem Constants
    public static final int ARM_CAN_ID = 21;
    public static final int ARM_STARTSWITCH_DIO_ID = 2;
    public static final int ARM_ENDSWITCH_DIO_ID = 3;
    public static final int ARM_ENCODER_DIO_A = 0; 
    public static final int ARM_ENCODER_DIO_B = 1;
    public static final int PIVOT_MOTOR_CURRENT_LIMIT = 60;
    public static final double GOVERNOR_PERCENT = 0.3;
    public static final double ARM_MAX_OPERATION_TIME_SECONDS = 4;
    public static final double ARM_MAX_OPERATION_SPEED_POWER = 0.5; // -1.0 to 1.0
    public static final double ARM_PIVOT_PID_TOLERANCE = 5;
    public static final double ARM_PIVOT_PID_MAX_OUTPUT = 0.25;
    public static final double ARM_PIVOT_PID_MIN_OUTPUT = -0.25;
    public static final double ARM_PIVOT_PID_SET_POINT_VERTICAL = 600.0;
    public static final double ARM_PIVOT_PID_SET_POINT_DEPLOYED = 1200.0;
    public static final double ARM_PIVOT_PID_SET_POINT_STOWED = 0.0;
    
    // Returns a safe pivot speed based on a governor constant
    public static double GetSafePivotSpeed(double speed) {
      return speed * GOVERNOR_PERCENT;
    }

  }
}
