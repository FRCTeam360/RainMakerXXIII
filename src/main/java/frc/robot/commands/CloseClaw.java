// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Claw;

public class CloseClaw extends CommandBase {
  private Claw claw = Claw.getInstance();

  private boolean hitLimit;

  private Timer timer = new Timer();
  
  /** Creates a new CloseClaw. */
  public CloseClaw() {
    addRequirements(claw);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println(this.getClass().getSimpleName() + "started");
    timer.start();
    hitLimit = false;
  }


  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(claw.getCurrent() > 15){
      hitLimit= true;
    }
    if(hitLimit || timer.get() > 1){
      claw.adjustsClaw(-0.05);
    } else {
      claw.adjustsClaw(-0.3);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println(this.getClass().getSimpleName() + "finished");
    claw.stopClaw();
    timer.stop();
    timer.reset();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
