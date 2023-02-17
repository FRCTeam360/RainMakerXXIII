// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.XboxConstants;
import frc.robot.subsystems.Intake;

public class ManualIntake extends CommandBase {
  Intake intake = Intake.getInstance();
  XboxController operatorCont = new XboxController(XboxConstants.OPERATOR_CONTROLLER_PORT);
  /** Creates a new ManualIntake. */
  public ManualIntake() {
    addRequirements(intake);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(operatorCont.getLeftBumper()){
      intake.run(-0.5);
    } else if(operatorCont.getRightBumper()){
      intake.run(0.5);
    } else {
      intake.stop();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
