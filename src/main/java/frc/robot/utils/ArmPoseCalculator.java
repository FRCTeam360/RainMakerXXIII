// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

public class ArmPoseCalculator { //NEVER MAKE STATIC ! WILL BREAK THINGS !

  private static final double pivotHeight = 1.0;

  private Translation3d robotTrans = new Translation3d(); 
  private Translation3d targetTrans;

  /**
   * first index is the alliance (0 is blue, 1 is red),
   * second index is the row (0 is bottom, 1 is middle, 2 is top), and
   * third index is the col (0 is the field edge, 8 is closest to the loading zone)
   **/
  public Translation3d[][][] nodeCoordinates = new Translation3d[2][3][9];
  private double[] yCoordinates = new double[] {0, 1, 2, 3, 4, 5, 6, 7, 8}; //y coordinates for all the nodes, starting at the field edge
  private double[] zCoordinatesCones = new double[] {1, 2, 3}; //z coordinates for all the cones, bottom -> top
  private double[] zCoordinatesCubes = new double[] {0.5, 1.5, 2.5}; //z coordinates for all the cubes, bottom -> top

  private final int blue = 0;
  private final int red = 1;

  private final int bot = 0;
  private final int mid = 1;
  private final int top = 2;

  /** Creates a new ArmPoseCalculator. */
  public ArmPoseCalculator() {
    setUp();
  }

  public void setUp() {
    for (int col = 0; col < 9; col++) { //sets all of the nodes going across all 3 grids to the same x and z values (all arbitrary)
      if (col == 1 || col == 4 || col == 7) {
        continue;
      }
      
      nodeCoordinates[blue][top][col] = new Translation3d(1, yCoordinates[col], zCoordinatesCones[2]); //moving across the field left to right
      nodeCoordinates[blue][mid][col] = new Translation3d(2, yCoordinates[col], zCoordinatesCones[1]);
      nodeCoordinates[blue][bot][col] = new Translation3d(3, yCoordinates[col], zCoordinatesCones[0]);

      nodeCoordinates[red][bot][col] = new Translation3d(4, yCoordinates[col], zCoordinatesCones[0]);
      nodeCoordinates[red][mid][col] = new Translation3d(5, yCoordinates[col], zCoordinatesCones[1]);
      nodeCoordinates[red][top][col] = new Translation3d(6, yCoordinates[col], zCoordinatesCones[2]);
    }

    for (int col = 1; col <= 7; col += 3) { //new translation3d w a diff z value, same x and y values tho (REMEMBER TO UPDATE HERE TOO)
      nodeCoordinates[blue][top][col] = new Translation3d(1, yCoordinates[col], zCoordinatesCubes[2]);
      nodeCoordinates[blue][mid][col] = new Translation3d(2, yCoordinates[col], zCoordinatesCubes[1]);
      nodeCoordinates[blue][bot][col] = new Translation3d(3, yCoordinates[col], zCoordinatesCubes[0]); 
      
      nodeCoordinates[red][bot][col] = new Translation3d(4, yCoordinates[col], zCoordinatesCubes[0]);
      nodeCoordinates[red][mid][col] = new Translation3d(5, yCoordinates[col], zCoordinatesCubes[1]);
      nodeCoordinates[red][top][col] = new Translation3d(6, yCoordinates[col], zCoordinatesCubes[2]);
    }
  }
  
  public void setRobotTrans(Translation3d trans){
    robotTrans = trans;
  }

  public void setRobotPose(Pose2d pose){
    setRobotTrans(new Translation3d(pose.getX(), pose.getY(), pivotHeight));
  }

  public void setTargetTrans(Translation3d trans){
    targetTrans = trans;
  }

   /**
   * first index is the alliance (0 is blue, 1 is red)
   * second index is the row (0 is bottom, 1 is middle, 2 is top)
   * third index is the col (0 is the field wall, 8 is closest to the loading zone)
   **/
  public void setNode(Translation3d coordinates){
    setTargetTrans(coordinates);
  }

  public Translation3d getTransform(){
    System.out.println("m_x "+ targetTrans.getX());
    System.out.println("m_y "+ targetTrans.getY());

    System.out.println("m_z "+ targetTrans.getZ());

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

  public double getTurretRotation(){ //TODO: gotta subtract from the drivetrain angle
    return Math.toDegrees(Math.atan(getY()/getX()));
  }

  public double get2dDistance(){
    return Math.sqrt((getX() * getX()) + (getY() * getY()));
  }

  public double getElevationAngleDegrees(){
    return Math.toDegrees(Math.atan(getZ() / get2dDistance()));
  }

  public double getExtendDistance(){
    return Math.sqrt((getX() * getX()) + (getY() * getY()) + (getZ() * getZ()));
  }
}
