// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems;

import com.ctre.phoenix.sensors.WPI_Pigeon2;
// import com.kauailabs.navx.frc.AHRS;
import com.swervedrivespecialties.swervelib.Mk4iSwerveModuleHelper;
import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;
import com.swervedrivespecialties.swervelib.SwerveModule;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANIds.CANivore;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static frc.robot.Constants.*;

import java.util.Objects;

public class DriveTrain extends SubsystemBase {
  private final Field2d field = new Field2d();
  private static DriveTrain instance;
  public static final double MAX_VOLTAGE = 12.0;

  private static final double ADJUSTMENT_FACTOR = 0.1;

  private PIDController pitchController = new PIDController(0.1, 0, 0);
  private PIDController rollController = new PIDController(0.1, 0, 0);

  public static Rotation2d[] stateAngles = { new Rotation2d(0.0), new Rotation2d(0.0), new Rotation2d(0.0),
      new Rotation2d(0.0) };
  public static final double MAX_VELOCITY_METERS_PER_SECOND = 6380.0 / 60.0
      * SdsModuleConfigurations.MK4I_L1.getDriveReduction() * SdsModuleConfigurations.MK4I_L1.getWheelDiameter()
      * Math.PI;

  public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = MAX_VELOCITY_METERS_PER_SECOND /
      Math.hypot(SwerveConstants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0, SwerveConstants.DRIVETRAIN_WHEELBASE_METERS / 2.0);

  public final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
      // Front left
      new Translation2d(SwerveConstants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0,
          SwerveConstants.DRIVETRAIN_WHEELBASE_METERS / 2.0),
      // Front right
      new Translation2d(SwerveConstants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0,
          -SwerveConstants.DRIVETRAIN_WHEELBASE_METERS / 2.0),
      // Back left
      new Translation2d(-SwerveConstants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0,
          SwerveConstants.DRIVETRAIN_WHEELBASE_METERS / 2.0),
      // Back right
      new Translation2d(-SwerveConstants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0,
          -SwerveConstants.DRIVETRAIN_WHEELBASE_METERS / 2.0));

  private final WPI_Pigeon2 m_pigeon = new WPI_Pigeon2(CANivore.DRIVETRAIN_PIGEON_ID, SwerveConstants.CANBUS);
  private final Limelight ll = Limelight.getInstance();

  private final SwerveDriveOdometry odometry;

  private Pose2d pose;

  private ChassisSpeeds currentVelocity = new ChassisSpeeds();

  private final SwerveModule m_frontLeftModule;
  private final SwerveModule m_frontRightModule;
  private final SwerveModule m_backLeftModule;
  private final SwerveModule m_backRightModule;

  public static DriveTrain getInstance() {
    if (instance == null) {
      instance = new DriveTrain();
    }
    return instance;
  }

  /** Creates a new DriveTrain. */
  public DriveTrain() {
    ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");
    m_frontLeftModule = Mk4iSwerveModuleHelper.createFalcon500(
        tab.getLayout("Front Left Module", BuiltInLayouts.kList)
            .withSize(2, 4)
            .withPosition(0, 0),
        Mk4iSwerveModuleHelper.GearRatio.L1,
        CANivore.FRONT_LEFT_MODULE_DRIVE_MOTOR,
        CANivore.FRONT_LEFT_MODULE_STEER_MOTOR,
        CANivore.FRONT_LEFT_MODULE_STEER_ENCODER,
        SwerveConstants.FRONT_LEFT_MODULE_STEER_OFFSET);
    System.out.println("module: " + m_frontLeftModule.toString());
    m_frontRightModule = Mk4iSwerveModuleHelper.createFalcon500(
        tab.getLayout("Front Right Module", BuiltInLayouts.kList)
            .withSize(2, 4)
            .withPosition(2, 0),
        Mk4iSwerveModuleHelper.GearRatio.L1,
        CANivore.FRONT_RIGHT_MODULE_DRIVE_MOTOR,
        CANivore.FRONT_RIGHT_MODULE_STEER_MOTOR,
        CANivore.FRONT_RIGHT_MODULE_STEER_ENCODER,
        SwerveConstants.FRONT_RIGHT_MODULE_STEER_OFFSET);

    m_backLeftModule = Mk4iSwerveModuleHelper.createFalcon500(
        tab.getLayout("Back Left Module", BuiltInLayouts.kList)
            .withSize(2, 4)
            .withPosition(4, 0),
        Mk4iSwerveModuleHelper.GearRatio.L1,
        CANivore.BACK_LEFT_MODULE_DRIVE_MOTOR,
        CANivore.BACK_LEFT_MODULE_STEER_MOTOR,
        CANivore.BACK_LEFT_MODULE_STEER_ENCODER,
        SwerveConstants.BACK_LEFT_MODULE_STEER_OFFSET);

    m_backRightModule = Mk4iSwerveModuleHelper.createFalcon500(
        tab.getLayout("Back Right Module", BuiltInLayouts.kList)
            .withSize(2, 4)
            .withPosition(6, 0),
        Mk4iSwerveModuleHelper.GearRatio.L1,
        CANivore.BACK_RIGHT_MODULE_DRIVE_MOTOR,
        CANivore.BACK_RIGHT_MODULE_STEER_MOTOR,
        CANivore.BACK_RIGHT_MODULE_STEER_ENCODER,
        SwerveConstants.BACK_RIGHT_MODULE_STEER_OFFSET);

    odometry = new SwerveDriveOdometry(m_kinematics, m_pigeon.getRotation2d(), getModulePositions());
    // SmartDashboard.putData("Field", field);
  }

  public void zeroGyroscope() {
    // FIXME Remove if you are using a Pigeon
    // m_pigeon.setFusedHeading(0.0);
    m_pigeon.setYaw(0.0);

    // FIXME Uncomment if you are using a NavX
    // m_navx.zeroYaw();
  }

  public Rotation2d getGyroscopeRotation() {
    // FIXME Remove if you are using a Pigeon
    // return Rotation2d.fromDegrees(m_pigeon.getFusedHeading());

    return m_pigeon.getRotation2d();

    // // FIXME Uncomment if you are using a NavX
    // if (m_navx.isMagnetometerCalibrated()) {
    // // We will only get valid fused headings if the magnetometer is calibrated
    // return Rotation2d.fromDegrees(m_navx.getFusedHeading());
    // }

    // // We have to invert the angle of the NavX so that rotating the robot
    // // counter-clockwise makes the angle increase.
    // return Rotation2d.fromDegrees(360.0 - m_navx.getYaw());
  }

  public void adjustAnglePosition() {
    drive(
        new ChassisSpeeds(
            // m_pigeon.getPitch() * ADJUSTMENT_FACTOR,
            // m_pigeon.getRoll() * ADJUSTMENT_FACTOR,
            pitchController.calculate(m_pigeon.getPitch(), 0),
            rollController.calculate(m_pigeon.getRoll(), 0),
            0));
  }

  public void drive(ChassisSpeeds chassisSpeeds) {
    setStates(m_kinematics.toSwerveModuleStates(chassisSpeeds));
  }

  public void setStates(SwerveModuleState[] states) {
    SwerveDriveKinematics.desaturateWheelSpeeds(states, MAX_VELOCITY_METERS_PER_SECOND);
    boolean speedIsZero = false;
    for (int i = 0; i < states.length; i++) {
      if (states[i].speedMetersPerSecond == 0) {
        speedIsZero = true;
      } else {
        speedIsZero = false;
        break;
      }
    }
    if (!speedIsZero) {
      m_frontLeftModule.set(states[0].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
          states[0].angle.getRadians());
      m_frontRightModule.set(states[1].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
          states[1].angle.getRadians());
      m_backLeftModule.set(states[2].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
          states[2].angle.getRadians());
      m_backRightModule.set(states[3].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
          states[3].angle.getRadians());
      for (int i = 0; i < states.length; i++) {
        stateAngles[i] = states[i].angle;
      }
    } else {
      // return angles with no speed
      m_frontLeftModule.set(0.0, stateAngles[0].getRadians());
      m_frontRightModule.set(0.0, stateAngles[1].getRadians());
      m_backLeftModule.set(0.0, stateAngles[2].getRadians());
      m_backRightModule.set(0.0, stateAngles[3].getRadians());
    }
    pose = odometry.update(getGyroscopeRotation(), getModulePositions());
  }

  public SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition frontLeftPosition = new SwerveModulePosition(m_frontLeftModule.getDriveDistance(),
        new Rotation2d(m_frontLeftModule.getSteerAngle()));
    SwerveModulePosition frontRightPosition = new SwerveModulePosition(m_frontRightModule.getDriveDistance(),
        new Rotation2d(m_frontRightModule.getSteerAngle()));
    SwerveModulePosition backLeftPosition = new SwerveModulePosition(m_backLeftModule.getDriveDistance(),
        new Rotation2d(m_backLeftModule.getSteerAngle()));
    SwerveModulePosition backRighPosition = new SwerveModulePosition(m_backRightModule.getDriveDistance(),
        new Rotation2d(m_backRightModule.getSteerAngle()));

    SwerveModulePosition[] positions = { frontLeftPosition, frontRightPosition, backLeftPosition, backRighPosition };

    return positions;
  }

  public Pose2d getPose() {
    return pose;
  }

  public SwerveDriveKinematics getKinematics() {
    return m_kinematics;
  }

  public void setPose(Pose2d pose) {
    odometry.resetPosition(getGyroscopeRotation(), getModulePositions(), pose);
  }

  public Pose2d setPose(Pose3d pose) {
    double x = pose.getX();
    double y = pose.getY();
    double a = pose.getRotation().getZ();
    Rotation2d r = new Rotation2d(a);
    Pose2d p = new Pose2d(x, y, r);
    odometry.resetPosition(r, getModulePositions(), p);
    return p;
  }

  public void resetPose() {
    odometry.resetPosition(getGyroscopeRotation(), getModulePositions(), new Pose2d());
  }

  public ChassisSpeeds getCurrentVelocity() {
    return currentVelocity;
  }

  public void resetSteerEncoders() {
    m_frontLeftModule.resetSteerEncoder();
    m_frontRightModule.resetSteerEncoder();
    m_backLeftModule.resetSteerEncoder();
    m_backRightModule.resetSteerEncoder();
  }

  public CommandBase xOutCommand(){
    return run( () -> {
      m_frontLeftModule.set(0, Math.toRadians(45));
      m_frontRightModule.set(0, Math.toRadians(135));
      m_backLeftModule.set(0, Math.toRadians(135));
      m_backRightModule.set(0, Math.toRadians(45));});
    }

  public void xOut(){
    m_frontLeftModule.set(0, Math.toRadians(45));
    m_frontRightModule.set(0, Math.toRadians(135));
    m_backLeftModule.set(0, Math.toRadians(135));
    m_backRightModule.set(0, Math.toRadians(45));
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("pitch", m_pigeon.getPitch());
    SmartDashboard.putNumber("roll", m_pigeon.getRoll());
    ll.runVision();
    if(Objects.isNull(ll.getAveragePose())) {
      pose = odometry.update(getGyroscopeRotation(), getModulePositions());
    } else {
      pose = this.setPose(ll.getAveragePose());
    }
    field.setRobotPose(pose);
    
    SmartDashboard.putData("field", field);
    SmartDashboard.putNumber("x pos", odometry.getPoseMeters().getX());
    SmartDashboard.putNumber("y pos", odometry.getPoseMeters().getY());
    // This method will be called once per scheduler run
  }
}
