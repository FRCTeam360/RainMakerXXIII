// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Limelight extends SubsystemBase {
  private static Limelight instance;
  private NetworkTable lime = NetworkTableInstance.getDefault().getTable("limelight");
  private NetworkTableEntry tv = lime.getEntry("tv");
  private NetworkTableEntry tx = lime.getEntry("tx");
  private NetworkTableEntry ty = lime.getEntry("ty");
  private NetworkTableEntry ta = lime.getEntry("ta");
  private NetworkTableEntry tl = lime.getEntry("tl");
  private NetworkTableEntry botpose = lime.getEntry("botpose");
  private NetworkTableEntry snap = lime.getEntry("snapshot");
  private double[] botposeArray = new double[6]; 

  private Turret turret = Turret.getInstance();
  private DriveTrain driveTrain = DriveTrain.getInstance();

  /** Creates a new Limelight. */
  public Limelight() {
    botposeArray = botpose.getDoubleArray(botposeArray);
    snap.setValue(0);
  }

  public static Limelight getInstance() {
    if(instance == null) {
      instance = new Limelight();
    }
    return instance;
  }

  public double getTX() {
    return tx.getDouble(0.0);
  }

  public double getTY() {
    return ty.getDouble(0.0);
  }

  public double getTV() {
    return tv.getDouble(0.0);
  }

  public boolean hasValidTarget() {
    return getTV() == 1;
  }

  public boolean isOnTarget() {
    return Math.abs(getTX()) <=1 && hasValidTarget();
  }

  public Pose2d getLimelightPose() {
    double turretAngle = Math.abs(turret.getAngle()) > 360 ? Math.signum(turret.getAngle())*(Math.abs(turret.getAngle()-360)) : turret.getAngle(); //for reference, this is "on god"
    double botAngle = botposeArray[4] - turretAngle; 
    Rotation2d r = new Rotation2d(Math.toRadians(botAngle));
    Pose2d p = new Pose2d(botposeArray[0]-8.27, botposeArray[1]-4.01, r); //TODO dunno if we have to account for the limelight offset in code or just in the pipeline. thx brandon for the docs T_T
    return p;
  }

  public void updateRobotPose() {
    driveTrain.setPose(getLimelightPose());
  }

  public void snapshot(boolean takeShot) {
    if(takeShot) {
      snap.setValue(1);
    } else {
      snap.setValue(0);
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
