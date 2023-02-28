// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Claw;

public class OpenClawCube extends CommandBase {
  private final Claw claw = Claw.getInstance();

  boolean shouldEnd;
  

  /** Creates a new OpenClaw. */
  public OpenClawCube(boolean shouldEnd) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(claw);
    this.shouldEnd = shouldEnd;
  }

  public OpenClawCube() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(claw);
    this.shouldEnd = true;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    claw.setPosition(90);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
      claw.stopClaw();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return shouldEnd && Math.abs(claw.getAbsoluteAngle() - 90.0) < 3;
  }
}
