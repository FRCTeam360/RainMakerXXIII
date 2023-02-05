// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import java.util.LinkedList;
import java.util.zip.ZipException;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

//LL practice bot up: .477
//LL practice bot forward: .045
//LL practice bot right: -.235 

public class Limelight{
  private static Limelight instance;
  private NetworkTable lime = NetworkTableInstance.getDefault().getTable("limelight");
  private NetworkTableEntry tv = lime.getEntry("tv");
  private NetworkTableEntry tx = lime.getEntry("tx");
  private NetworkTableEntry ty = lime.getEntry("ty");
  private NetworkTableEntry ta = lime.getEntry("ta");
  private NetworkTableEntry tl = lime.getEntry("tl");
  private NetworkTableEntry botpose = lime.getEntry("botpose_wpiblue");
  private NetworkTableEntry snap = lime.getEntry("snapshot");
  private double[] botposeArray = new double[6]; 
  private Pose3d averagePose = null;
  /**private double zX;
  private double zY;
  private double zZ;**/
  private int periodicCycles = 0;

  private LinkedList<Pose3d> poses = new LinkedList<>();

  private Turret turret = Turret.getInstance();

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

  public Pose3d getAveragePose() {
    return averagePose;
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



  public void updateLimelightPose() {
    double turretAngle = Math.abs(turret.getAngleRelativeToRobot()) > 360 ? Math.signum(turret.getAngleRelativeToRobot())*(Math.abs(turret.getAngleRelativeToRobot()-360)) : turret.getAngleRelativeToRobot(); //for reference, this is "on god"
    double botAngle = botposeArray[4] - turretAngle; 
    Rotation2d r = new Rotation2d(Math.toRadians(botAngle));
    double x = botposeArray[0]-8.27;
    double y = botposeArray[1]-4.01;
    Pose2d p = new Pose2d(x, y, r); //TODO dunno if we have to account for the limelight offset in code or just in the pipeline. thx brandon for the docs T_T
    if(x>0 && y>0 && x<16.54 && y<8.02) {
      //driveTrain.setPose(p);
    }
  }

  /**private void zScore() {
    double meanX = 0;
    double meanY = 0;
    double meanZ = 0;
    double sdX = 0;
    double sdY = 0;
    double sdZ = 0;
    for(Pose3d pose : poses) {
      meanX += pose.getX();
      meanY += pose.getY();
      meanZ += pose.getZ();
    }
    meanX = meanX / poses.size();
    meanY = meanY / poses.size();
    meanZ = meanZ / poses.size();

    for(Pose3d pose : poses) {
      sdX += Math.pow((pose.getX() - meanX), 2);
      sdY += Math.pow((pose.getY() - meanY), 2);
      sdZ += Math.pow((pose.getZ() - meanZ), 2);
    }

    sdX = Math.sqrt(sdX / (poses.size() - 1));
    sdY = Math.sqrt(sdY / (poses.size() - 1));
    sdZ = Math.sqrt(sdZ / (poses.size() - 1));

    for(Pose3d pose : poses) {
      double zX = (pose.getX() - meanX) / sdX;
      double zY = (pose.getY() - meanX) / sdY;
      double zZ = (pose.getZ() - meanX) / sdZ;
    }
  }**/

  public Pose3d averagePositions(LinkedList<Pose3d> lastPoses) {
    double xSum = 0.0;
    double ySum = 0.0;
    double zSum = 0.0;
    double rollSum = 0.0;
    double pitchSum = 0.0;
    double yawSum = 0.0;
    for(int i = 0; i>8; i++) {
      xSum += lastPoses.get(i).getX();
      ySum += lastPoses.get(i).getY();
      zSum += lastPoses.get(i).getZ();
      rollSum += lastPoses.get(i).getRotation().getX();
      pitchSum += lastPoses.get(i).getRotation().getY();
      yawSum += lastPoses.get(i).getRotation().getZ();
    }
    double xAvg = xSum/8.0;
    double yAvg = ySum/8.0;
    double zAvg = zSum/8.0;
    double rollAvg = rollSum/8.0;
    double pitchAvg = pitchSum/8.0;
    double yawAvg = yawSum/8.0;
    Rotation3d r = new Rotation3d(rollAvg, pitchAvg, yawAvg);
    Pose3d p = new Pose3d(xAvg, yAvg, zAvg, r);
    return p;
  }

  public void snapshot(boolean takeShot) {
    if(takeShot) {
      snap.setValue(1);
    } else {
      snap.setValue(0);
    }
  }

  public void runVision() {
    if(getTV() == 1) {
      periodicCycles = 10;
      Rotation3d r = new Rotation3d(botposeArray[3], botposeArray[4], botposeArray[5]);
      Pose3d tempPose = new Pose3d(botposeArray[0], botposeArray[1], botposeArray[2], r);
      System.out.println(tempPose);
      if(poses.isEmpty() || !tempPose.equals(poses.getFirst())) {
        poses.addFirst(tempPose);
      }
      System.out.println("pose.size: " + poses.size());
      if(poses.size()>8) {
        poses.removeLast();
      }
      if(poses.size()==8) {
        averagePose = averagePositions(poses);
      }
      else{
        averagePose = null;
      }
    } else {
      periodicCycles--;
    }
    if(periodicCycles == 0) {
      poses.clear();
      averagePose = null;
    }
    botposeArray = botpose.getDoubleArray(botposeArray);
  }
}