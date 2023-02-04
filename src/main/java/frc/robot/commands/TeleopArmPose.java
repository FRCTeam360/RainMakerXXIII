// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmExtend;
import frc.robot.subsystems.ArmTilt;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Turret;
import frc.robot.utils.ArmPoseCalculator;

public class TeleopArmPose extends CommandBase {

  private static ArmExtend armExtend = ArmExtend.getInstance();
  private static ArmTilt armTilt = ArmTilt.getInstance();
  private static Turret turret = Turret.getInstance();
  private static DriveTrain driveTrain = DriveTrain.getInstance();

  private static ArmPoseCalculator calculator;

  private static XboxController driverCont = new XboxController(0);
  private static XboxController operatorCont = new XboxController(1);

  private int alliance;
  private int row;
  private int col;

  /** Creates a new AutoArmPose. */
  public TeleopArmPose() {
    addRequirements(armExtend, armTilt, turret);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    calculator = new ArmPoseCalculator();
  }

  // Called every time the scheduler runs while the command is scheduled.
  
  @Override
  public void execute() {
    calculator.setRobotPose(driveTrain.getPose());
    chooseNode();
    calculator.setNode(calculator.nodeCoordinates[alliance][row][col]);
  }

  public void chooseNode() {
    if (DriverStation.getAlliance() == Alliance.Blue) {
      alliance = 0;
    } else if (DriverStation.getAlliance() == Alliance.Red) {
      alliance = 1;
    }

    //operator chooses the row
    if (operatorCont.getYButton()) {
      row = 2;
    } else if (operatorCont.getAButton()) {
      row = 0;
    } else {
      row = 1;
    }

    //operator chooses if its the left, mid, or right column in the grid
    if (operatorCont.getXButton()) {
      col = 0;
    } else if (operatorCont.getBButton()) {
      col = 2;
    } else {
      col = 1;
    }

    //driver chooses grid
    if (driverCont.getAButton()){
      col += 3;
    } else if (driverCont.getXButton()){
      col += 6;
    }

    calculator.setNode(calculator.nodeCoordinates[alliance][row][col]);
    armTilt.setAngle(calculator.getElevationAngleDegrees());
    armExtend.setPosition(calculator.getExtendDistance());
    turret.angleTurn(calculator.getTurretRotation());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
