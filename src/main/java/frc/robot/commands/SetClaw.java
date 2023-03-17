// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Claw;

public class SetClaw extends CommandBase {
  private final Claw claw = Claw.getInstance();
  private double angle;
  private boolean shouldEnd;
  /** Creates a new OpenClawCone. */
  public SetClaw(double angle, boolean shouldEnd) {
    addRequirements(claw);
    this.angle = angle;
    this.shouldEnd = shouldEnd;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  public SetClaw(double angle){
    this(angle, false);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    claw.setPosition(angle);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    claw.stopClaw();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return shouldEnd ? Math.abs(claw.getAbsoluteAngle() - angle) < 3 : false;
  }
}
