// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class RunIntakeReversed extends CommandBase {
  private final Intake intake = Intake.getInstance();

  /** Creates a new RunIntakeReversed. */
  public RunIntakeReversed() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println(this.getClass().getSimpleName() + " started");}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intake.run(-0.3);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.stop();
    System.out.println(this.getClass().getSimpleName() + " finished");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
