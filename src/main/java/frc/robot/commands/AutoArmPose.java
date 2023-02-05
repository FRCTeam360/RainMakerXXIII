// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmExtend;
import frc.robot.subsystems.ArmTilt;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Turret;
import frc.robot.utils.ArmPoseCalculator;

public class AutoArmPose extends CommandBase {
  private static ArmTilt armTilt = ArmTilt.getInstance();
  private static ArmExtend armExtend = ArmExtend.getInstance();
  private static Turret turret = Turret.getInstance();
  private static DriveTrain driveTrain = DriveTrain.getInstance();

  private static ArmPoseCalculator calculator;

  private int row;
  private int col;
  private int alliance;

  /** Creates a new AutoArmPose. */
  public AutoArmPose(int row, int col) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(armTilt, armExtend, turret);
    this.row = row;
    this.col = col;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    calculator = new ArmPoseCalculator();

    if (DriverStation.getAlliance() == Alliance.Blue) {
      alliance = 0;
    } else if (DriverStation.getAlliance() == Alliance.Red) {
      alliance = 1;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println("in autoarmpose");
    calculator.setRobotPose(driveTrain.getPose());
    calculator.setNode(calculator.nodeCoordinates[alliance][row][col]);
    armTilt.smartTilt(calculator.getActualElevationAngleDegrees());
    //armExtend.setPosition(calculator.getExtendDistance());
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
