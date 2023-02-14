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

  public enum RobotType {
    DRAFT, PRACTICE
  }

  public static final RobotType robotType = RobotType.PRACTICE;

  public static RobotType getRobotType() {
    return robotType;
  }
  
  public static class SwerveConstants {
    public static final String CANBUS = "Default Name";
    public static final double DRIVETRAIN_TRACKWIDTH_METERS = 0.62865; 
    public static final double DRIVETRAIN_WHEELBASE_METERS = 0.62865;
    public static final double DRAFT_BACK_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(92.72); //93.07 
    public static final double DRAFT_BACK_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(96.06); //94.48 //96.32 
    public static final double DRAFT_FRONT_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(189.93); //189.49 
    public static final double DRAFT_FRONT_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(271.75); //271.14 //0.97 

    public static final double PRACTICE_BACK_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(170.947265625); //170.947265625 
    public static final double PRACTICE_BACK_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(163.30078125); //335.0390625 
    public static final double PRACTICE_FRONT_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(210.41015625); //210.41015625 
    public static final double PRACTICE_FRONT_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(294.43359375); //294.43359375 

    public static double getBackRightModuleSteerOffset() {
      if(robotType == RobotType.DRAFT) {
        return SwerveConstants.DRAFT_BACK_RIGHT_MODULE_STEER_OFFSET;
      } else if(robotType == RobotType.PRACTICE) {
        return SwerveConstants.PRACTICE_BACK_RIGHT_MODULE_STEER_OFFSET;
      } else {
        return 0.0;
      }
    }
  
    public static double getBackLeftModuleSteerOffset() {
      if(robotType == RobotType.DRAFT) {
        return SwerveConstants.DRAFT_BACK_LEFT_MODULE_STEER_OFFSET;
      } else if(robotType == RobotType.PRACTICE) {
        return SwerveConstants.PRACTICE_BACK_LEFT_MODULE_STEER_OFFSET;
      } else {
        return 0.0;
      }
    }
  
    public static double getFrontRightModuleSteerOffset() {
      if(robotType == RobotType.DRAFT) {
        return SwerveConstants.DRAFT_FRONT_RIGHT_MODULE_STEER_OFFSET;
      } else if(robotType == RobotType.PRACTICE) {
        return SwerveConstants.PRACTICE_FRONT_RIGHT_MODULE_STEER_OFFSET;
      } else {
        return 0.0;
      }
    }
  
    public static double getFrontLeftModuleSteerOffset() {
      if(robotType == RobotType.DRAFT) {
        return SwerveConstants.DRAFT_FRONT_LEFT_MODULE_STEER_OFFSET;
      } else if(robotType == RobotType.PRACTICE) {
        return SwerveConstants.PRACTICE_FRONT_LEFT_MODULE_STEER_OFFSET;
      } else {
        return 0.0;
      }
    }
  }

  public static class XboxConstants {
    public static final int DRIVER_CONTROLLER_PORT = 0;
    public static final int OPERATOR_CONTROLLER_PORT = 1;
  }

  public static class CANIds {
    //0 is roborio id
    public final static int PDH_ID = 1;

    public final static int TURRET_ID = 2;
    public final static int TILT_FOLLOW_ID = 3;
    public final static int TILT_LEAD_ID = 4;
    public final static int EXTEND_LEAD_ID = 5;
    public final static int EXTEND_FOLLOW_ID = 6;
    public final static int CLAW_GRIP_ID = 7;
    public final static int INTAKE_ID = 8;

    public static class CANivore {
        public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 12; 
        public static final int FRONT_LEFT_MODULE_STEER_MOTOR = 10; 
        public static final int FRONT_LEFT_MODULE_STEER_ENCODER = 11; 

        public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 3; 
        public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 1; 
        public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 2; 

        public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 9; 
        public static final int BACK_LEFT_MODULE_STEER_MOTOR = 7; 
        public static final int BACK_LEFT_MODULE_STEER_ENCODER = 8;

        public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 6;
        public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 4; 
        public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 5; 

        public static final int DRIVETRAIN_PIGEON_ID = 13; 

    }
  }

  public static class DigitalIOIds {
    public static final int TURRET_LIMIT_SWITCH_ID = 0;
  }
}
