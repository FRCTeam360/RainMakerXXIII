// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

public class ArmPoseCalculator {

  private Translation3d robotTrans; 
  private Translation3d targetTrans;

  /** Creates a new ArmPoseCalculator. */
  public ArmPoseCalculator() {}

  public void setRobotPose(Translation3d trans){
    robotTrans = trans;
  }

  public void setTargetPose(Translation3d trans){
    targetTrans = trans;
  }

  public Translation3d getTransform(){
    return targetTrans.minus(robotTrans);
  }

  public double getX(){
    return getTransform().getX();
  }

  public double getY(){
    return getTransform().getY();
  }

  public double getZ(){
    return getTransform().getZ();
  }

  // public Rotation3d getRotation3d(){
  //   return getTransform().getRotation();
  // }

  // public Rotation2d getRotation2d(){
  //   return getRotation3d().toRotation2d();
  // }

  // public double getYawDegrees(){
  //   return getRotation2d().getDegrees();
  // }

  public double getTurretRotation(){
    return Math.toDegrees(Math.atan(getY()/getX()));
  }

  // public double getElevationAngleRadians(){
  //   return Math.sqrt(Math.pow(getRotation3d().getX(), 2) + Math.pow(getRotation3d().getY(), 2));
  // }

  public double get2dDistance(){
    return Math.sqrt((getX() * getX()) + (getY() * getY()));
  }

  public double getElevationAngleDegrees(){
    return Math.toDegrees(Math.atan(getZ() / get2dDistance()));
  }

  // public double getElevationAngleDegrees(){
  //   return Math.toDegrees(getElevationAngleRadians());
  // }

  // public double getExtendDistance(){
  //   return Math.sqrt(Math.pow(getX(), 2) + Math.pow(getY(), 2) + Math.pow(getZ(), 2));
  // }

  public double getExtendDistance(){
    return Math.sqrt((getX() * getX()) + (getY() * getY()) + (getZ() * getZ()));
  }
}
