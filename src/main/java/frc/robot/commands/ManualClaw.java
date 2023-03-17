// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Claw;

public class ManualClaw extends CommandBase {
  private Claw claw = Claw.getInstance();

  private boolean hitLimit = false;

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
      if(claw.getCurrent() > 15){
        hitLimit= true;
      }
      // if(hitLimit){
      //   claw.adjustsClaw(-0.05);
      // } else {
        claw.adjustsClaw(operatorCont.getLeftTriggerAxis() * -0.3);
      // }
    } else if (Math.abs(operatorCont.getRightTriggerAxis()) > 0.1) { //open
      claw.adjustsClaw(operatorCont.getRightTriggerAxis() *0.3);
      hitLimit = false;
    } else {
      claw.stopClaw();
      hitLimit = false;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    claw.stopClaw();
    hitLimit = false;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
