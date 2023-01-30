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
  
  public static class SwerveConstants {
    public static final String CANBUS = "Default Name";
    public static final double DRIVETRAIN_TRACKWIDTH_METERS = 0.62865; 
    public static final double DRIVETRAIN_WHEELBASE_METERS = 0.62865;
    public static final double BACK_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(92.72); //93.07 // FIXME Measure and set back right steer offset
    public static final double BACK_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(96.06); //94.48 //96.32 // FIXME Measure and set back left steer offset
    public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(189.93); //189.49 // FIXME Measure and set front right steer offset
    public static final double FRONT_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(271.75); //271.14 //0.97 FIXME Measure and set front left steer offset
  }

  public static class XboxConstants {
    public static final int DRIVER_CONTROLLER_PORT = 0;
    public static final int OPERATOR_CONTROLLER_PORT = 1;
  }

  public static class CANIds {
    //0 is roborio id
    public final static int PDH_ID = 1;
    public final static int PH_ID = 2;

    public final static int TURRET_ID = 3;
    public final static int TILT_LEAD_ID = 4;
    //public final static int TILT_FOLLOW_ID = 5;
    public final static int EXTEND_ID = 6;
    public final static int EXTEND_FOLLOW_ID = 7;

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

}
