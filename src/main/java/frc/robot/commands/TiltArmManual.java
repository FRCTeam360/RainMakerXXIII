// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.XboxConstants;
import frc.robot.subsystems.ArmTilt;

public class TiltArmManual extends CommandBase {
  private static ArmTilt tilt = ArmTilt.getInstance();

  private static XboxController operatorCont = new XboxController(XboxConstants.OPERATOR_CONTROLLER_PORT);
  
  /** Creates a new ArmTilt. */
  public TiltArmManual() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(tilt);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(Math.abs(operatorCont.getRightY()) >= 0.1) {
      tilt.adjustTilt(operatorCont.getRightY() * 0.1);
      } else {
        tilt.adjustTilt(0);
      }
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
