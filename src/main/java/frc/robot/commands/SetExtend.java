// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmExtend;

public class SetExtend extends CommandBase {
  private ArmExtend extend = ArmExtend.getInstance();
  
  private double distance;
  private boolean shouldEnd;
  /** Creates a new SetExtend. */
  public SetExtend(double distance, boolean shouldEnd) {
    this.distance = distance;
    this.shouldEnd = shouldEnd;
    addRequirements(extend);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println(this.getClass().getSimpleName() + "started");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    extend.setPosition(distance);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println(this.getClass().getSimpleName() + "finished");
    extend.adjustExtensionSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return shouldEnd ? Math.abs(extend.getExtendDistance() - distance) < 0.02 : false;
  }
}
