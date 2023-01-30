// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmExtend;
import frc.robot.subsystems.ArmTilt;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Turret;

public class AutoArmPose extends CommandBase {

  private static ArmExtend armExtend = ArmExtend.getInstance();
  private static ArmTilt armTilt = ArmTilt.getInstance();
  private static Turret turret = Turret.getInstance();
  private static DriveTrain driveTrain = DriveTrain.getInstance();

  /** Creates a new AutoArmPose. */
  public AutoArmPose() {
    addRequirements(armExtend, armTilt, turret);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
