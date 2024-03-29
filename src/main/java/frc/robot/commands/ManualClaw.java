// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Claw;

public class ManualClaw extends CommandBase {
  private Claw claw = Claw.getInstance();

  private XboxController operatorCont = new XboxController(1);
  /** Creates a new ManualClaw. */
  public ManualClaw() {
    addRequirements(claw);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(Math.abs(operatorCont.getLeftTriggerAxis()) > 0.1){ //close
      claw.adjustsClaw(operatorCont.getLeftTriggerAxis());
    } else if (Math.abs(operatorCont.getRightTriggerAxis()) > 0.1) { //open
      claw.adjustsClaw(operatorCont.getRightTriggerAxis() * -1.0);
    } else {
      claw.stopClaw();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    claw.stopClaw();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
