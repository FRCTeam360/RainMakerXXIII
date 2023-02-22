// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.subsystems.ArmExtend;
import frc.robot.subsystems.DriveTrain;

public class ArmPoseCalculator { //NEVER MAKE STATIC ! WILL BREAK THINGS !

  private ArmExtend extend = ArmExtend.getInstance();

  private static final double pivotHeight = 0.5334;

  private Translation3d robotTrans = new Translation3d(0, 0, pivotHeight); 
  private Translation3d targetTrans = new Translation3d();

  private final int cube = 0;
  private final int cone = 1;

  private final int blue = 0;
  private final int red = 1;

  private final int bot = 0;

  /**
   * first index is the alliance (0 is blue, 1 is red),
   * second index is the row (0 is bottom, 1 is middle, 2 is top), and
   * third index is the column (0 is the field edge, 8 is closest to the loading zone)
   **/
  public final Translation3d[][][] nodeCoordinates = new Translation3d[2][3][9];

  private double[] xCoordinatesBlue = new double[] {1.1608, 0.7954, 0.3629}; //x coordinates on the blue alliance, bottom -> top
  private double[] xCoordinatesRed = new double[] {15.3613, 15.7408, 16.1661}; //x coordinates on the red alliance, bottom -> top

  private double[] yCoordinates = new double[] {0.5127, 1.0705, 1.6303, 2.1891, 2.7479, 3.3067, 3.8655, 4.4243, 4.9831}; //y coordinates for all the nodes, starting at the field edge
  private double[] yCoordinatesHybrid = new double[] {0.4206, 5.0752}; //y coordinates for the bottom nodes in columns 0, 2, 3, 5, 6, and 8 (not centered)

  private double[] zCoordinatesCones = new double[] {0, 0.8652, 1.17}; //z coordinates for all the cones, bottom -> top
  private double[] zCoordinatesCubes = new double[] {0, 0.5223, 0.8263}; //z coordinates for all the cubes, bottom -> top


 private Translation3d[][] pieceCoordinates = new Translation3d[2][4]; //two alliances, four pieces each, field edge -> loading zone
 private double[] pieceCoordinatesX = new double[] {7.0615, 8.8163}; //blue, red
 private double[] pieceCoordinatesY = new double[] {0.9097, 2.1383, 3.3575, 4.5468}; //field edge -> loading zone
 private double[] pieceCoordinatesZ = new double[] {0, 1}; //TODO: NOT BE 0 AND 1 ! cube, cone

  private int[] notCentered = new int[] {0, 8}; //indexes of the noncentered bottom nodes

  /** Creates a new ArmPoseCalculator. */
  public ArmPoseCalculator() {
    setUp();

  }


  public void setUp() {
    
    for(int all = 0; all < 2; all++) { //updates pieceCoordinates !
      for (int piece = 0; piece < 4; piece++) {
        pieceCoordinates[all][piece] = new Translation3d( pieceCoordinatesX[all], pieceCoordinatesY[piece], pieceCoordinatesZ[cube]);
      }
    }
    
    for (int col = 0; col < 9; col++) { //starts at field edge, moving towards the loading zone
      if (col == 1 || col == 4 || col == 7) { //skips the cube nodes
        continue;
      }

      for (int row = 0; row < 3; row++) { //bottom to top
        if ((col == 0 || col == 8) && row == 0) { //skips noncentered bottom nodes 
          continue;
        }

        nodeCoordinates[blue][row][col] = new Translation3d (xCoordinatesBlue[row], yCoordinates[col], zCoordinatesCones[row]);
        nodeCoordinates[red][row][col] = new Translation3d (xCoordinatesRed[row], yCoordinates[col], zCoordinatesCones[row]);
      }
    }

    for (int col = 1; col <= 7; col += 3) { //updates the cube nodes
      for (int row = 0; row < 3; row++) {
        nodeCoordinates[blue][row][col] = new Translation3d (xCoordinatesBlue[row], yCoordinates[col], zCoordinatesCubes[row]);
        nodeCoordinates[red][row][col] = new Translation3d (xCoordinatesRed[row], yCoordinates[col], zCoordinatesCubes[row]);
      }
    }

    for (int i = 0; i < 2; i++) { //updates the noncentered bottom nodes (which are never cubes)
      int index = notCentered[i];
      nodeCoordinates[blue][bot][index] = new Translation3d (xCoordinatesBlue[bot], yCoordinatesHybrid[i], zCoordinatesCubes[bot]);
      nodeCoordinates[red][bot][index] = new Translation3d (xCoordinatesRed[bot], yCoordinatesHybrid[i], zCoordinatesCubes[bot]);
    }
  }

  public Translation3d getNode(int all, int row, int col) {
    return nodeCoordinates[all][row][col];
  }

  public Translation3d getPiece(int all, int piece) {
    return pieceCoordinates[all][piece];
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
   * first index is the alliance (0 is blue, 1 is red),
   * second index is the row (0 is bottom, 1 is middle, 2 is top), and
   * third index is the column (0 is the field wall, 8 is closest to the loading zone)
   **/
  public void setNode(Translation3d coordinates){
    setTargetTrans(coordinates);
  }

  public Translation3d getTransform(){
    // System.out.println("m_x "+ targetTrans.getX());
    // System.out.println("m_y "+ targetTrans.getY());

    // System.out.println("m_z "+ targetTrans.getZ());

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

  public double getTurretRotation(){ 
    if(getX() >= 0){
      return Math.toDegrees(Math.atan(getY()/getX())); //TODO fix math to account for atan range
    } else if(getY() >= 0){
      return Math.toDegrees(Math.atan(getY()/getX())) + 180;
    } else {
      return Math.toDegrees(Math.atan(getY()/getX())) - 180;
    }
  }

  public double getFieldRelativeTurretRotation(){
    return getTurretRotation() - DriveTrain.getInstance().getGyroscopeRotation().getDegrees();
  }

  public double get2dDistance(){
    return Math.sqrt((getX() * getX()) + (getY() * getY()));
  }

  public double getTheoreticalElevationAngleDegrees(){
    return Math.toDegrees(Math.atan(getZ() / get2dDistance()));
  }

  public double getExtendDistance(){
    return Math.sqrt((getX() * getX()) + (getY() * getY()) + (getZ() * getZ()));
  }

  public double getActualElevationAngleDegrees(){
    return Math.toDegrees(Math.asin(getZ() / extend.getDistanceFromPivot()));
  }
}

/*
 * blue x: 7.0615
 * red x: 8.8163
 * 
 * y one: 0.9097
 * y two: 2.1383
 * y three: 3.3575
 * y four: 4.5468
 */

  /*
   * blue bot x: 1.1608
   * blue mid x: 0.7954
   * blue top x: 0.3629
   *
   * red bot x: 15.3613
   * red mid x: 15.7408
   * red top x: 16.1661
   * 
   * cones mid z: 0.8652
   * cones top z: 1.17
   * 
   * cubes bot z: 0 
   * cubes mid z: 0.5223
   * cubes top z: 0.8263
   * 
   * y 0 bot: 0.4206
   * y 8 bot: 5.0752
   * 
   * y 0: 0.5127
   * y 1: 1.0705
   * y 2: 1.6303
   * y 3: 2.1891
   * y 4: 2.7479
   * y 5: 3.3067
   * y 6: 3.8655
   * y 7: 4.4243
   * y 8: 4.9831
   * 
   * 
    nodeCoordinates[blue][bot][0] = new Translation3d (xCoordinatesBlue[bot], yCoordinatesHybrid[0], zCoordinatesCubes[bot]);
    nodeCoordinates[blue][bot][2] = new Translation3d (xCoordinatesBlue[bot], yCoordinatesHybrid[2], zCoordinatesCubes[bot]);
    nodeCoordinates[blue][bot][3] = new Translation3d (xCoordinatesBlue[bot], yCoordinatesHybrid[3], zCoordinatesCubes[bot]);
    nodeCoordinates[blue][bot][5] = new Translation3d (xCoordinatesBlue[bot], yCoordinatesHybrid[5], zCoordinatesCubes[bot]);
    nodeCoordinates[blue][bot][6] = new Translation3d (xCoordinatesBlue[bot], yCoordinatesHybrid[6], zCoordinatesCubes[bot]);
    nodeCoordinates[blue][bot][8] = new Translation3d (xCoordinatesBlue[bot], yCoordinatesHybrid[8], zCoordinatesCubes[bot]);

    nodeCoordinates[red][bot][2] = new Translation3d (xCoordinatesRed[bot], yCoordinatesHybrid[2], zCoordinatesCubes[bot]);
    nodeCoordinates[red][bot][3] = new Translation3d (xCoordinatesRed[bot], yCoordinatesHybrid[3], zCoordinatesCubes[bot]);
    nodeCoordinates[red][bot][5] = new Translation3d (xCoordinatesRed[bot], yCoordinatesHybrid[5], zCoordinatesCubes[bot]);
    nodeCoordinates[red][bot][0] = new Translation3d (xCoordinatesRed[bot], yCoordinatesHybrid[0], zCoordinatesCubes[bot]);
    nodeCoordinates[red][bot][6] = new Translation3d (xCoordinatesRed[bot], yCoordinatesHybrid[6], zCoordinatesCubes[bot]);
    nodeCoordinates[red][bot][8] = new Translation3d (xCoordinatesRed[bot], yCoordinatesHybrid[8], zCoordinatesCubes[bot]);
   */