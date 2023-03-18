// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmTilt;

public class SetTilt extends CommandBase {
  /** Creates a new SetTilt. */
  private ArmTilt tilt = ArmTilt.getInstance();

  private double angle;
  private boolean shouldEnd;

  public SetTilt(double angle, boolean shouldEnd) {
    this.angle = angle;
    this.shouldEnd = shouldEnd;
    addRequirements(tilt);
  }

  public SetTilt(double angle){
    this(angle, false);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    tilt.setAngle(angle);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    tilt.adjustTilt(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return shouldEnd ? Math.abs(tilt.getAngle() - angle) < 2 : false;
  }
}
