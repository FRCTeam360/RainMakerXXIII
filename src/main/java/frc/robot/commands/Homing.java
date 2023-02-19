// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.XboxConstants;
import frc.robot.subsystems.ArmExtend;
import frc.robot.subsystems.ArmTilt;
import frc.robot.subsystems.Turret;

public class Homing extends CommandBase {
  private final ArmTilt tilt = ArmTilt.getInstance();
  private final ArmExtend extend = ArmExtend.getInstance();
  private final Turret turret = Turret.getInstance();

  private final XboxController operatorCont = new XboxController(XboxConstants.OPERATOR_CONTROLLER_PORT);

  /** Creates a new Homing. */
  public Homing() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(tilt, extend, turret);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (operatorCont.getYButtonPressed()) {
      tilt.setAngle(90);
      extend.setPosition(0.1);
      turret.setPosition(0);
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
