// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.XboxConstants;
import frc.robot.subsystems.ArmExtend;


public class ExtendArmManual extends CommandBase {

  private static ArmExtend extend = ArmExtend.getInstance();
  private static XboxController operatorCont = new XboxController(XboxConstants.OPERATOR_CONTROLLER_PORT);

  /** Creates a new ArmExtension. */
  public ExtendArmManual() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(extend);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(Math.abs(operatorCont.getLeftY()) >= 0.1) {
      extend.adjustExtension(-operatorCont.getLeftY() * 0.3);
    } else if(operatorCont.getLeftY() >= 0.1){
      extend.adjustExtension(operatorCont.getLeftY() * 0.3);
    } else {
      extend.adjustExtension(0);
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