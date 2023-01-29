// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import javax.xml.crypto.dsig.TransformService;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

public class ArmPoseCalculator {

  private Translation3d robotTrans; 
  private Translation3d targetTrans;

  private Translation3d[][][] nodeCoordinates = new Translation3d[2][3][9]; //alliances, rows, nodes
  private double[] yCoordinates = new double[] {0, 1, 2, 3, 4, 5, 6, 7, 8}; //y coordinates for all the nodes, starting at the wall

  /** Creates a new ArmPoseCalculator. */
  public ArmPoseCalculator() {

  }

  public void setUp() {
    for (int node = 0; node < 9; node++) { //sets all of the nodes going across all 3 grids to the same x and z values (all arbitrary)
      nodeCoordinates[0][2][node] = new Translation3d(1, yCoordinates[node], 3); //moving across the field left to right
      nodeCoordinates[0][1][node] = new Translation3d(2, yCoordinates[node], 2);
      nodeCoordinates[0][0][node] = new Translation3d(3, yCoordinates[node], 1);

      nodeCoordinates[1][0][node] = new Translation3d(4, yCoordinates[node], 1);
      nodeCoordinates[1][1][node] = new Translation3d(5, yCoordinates[node], 2);
      nodeCoordinates[1][2][node] = new Translation3d(6, yCoordinates[node], 3);
    }

    for (int node = 1; node <= 7; node += 3) { //new translation3d w a diff z value, same x and y values tho (REMEMBER TO UPDATE HERE TOO)
      nodeCoordinates[0][2][node] = new Translation3d(1, yCoordinates[node], 2.5);
      nodeCoordinates[0][1][node] = new Translation3d(2, yCoordinates[node], 1.5);
      nodeCoordinates[0][0][node] = new Translation3d(3, yCoordinates[node], 0.5); 
      
      nodeCoordinates[1][0][node] = new Translation3d(4, yCoordinates[node], 0.5);
      nodeCoordinates[1][1][node] = new Translation3d(5, yCoordinates[node], 1.5);
      nodeCoordinates[1][2][node] = new Translation3d(6, yCoordinates[node], 2.5);
    }
  }
  
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
