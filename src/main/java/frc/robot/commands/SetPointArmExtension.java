// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmExtend;

public class SetPointArmExtension extends CommandBase {
  /** Creates a new SetPointArmExtensions. */

  ArmExtend extend = ArmExtend.getInstance();

  public SetPointArmExtension() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(extend);
    SmartDashboard.putNumber("ArmExtendSetPoint", 0);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double setPosition = SmartDashboard.getNumber("ArmExtendSetPoint", 0);
    extend.setPosition(setPosition);

    SmartDashboard.putNumber("extend error", extend.getExtendDistance() - setPosition);
    
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
