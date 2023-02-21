// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmExtend;
import frc.robot.subsystems.ArmTilt;
import frc.robot.subsystems.Turret;
import frc.robot.utils.ArmPoseCalculator;

public class SetArmPose extends CommandBase {

  private final ArmExtend extend = ArmExtend.getInstance();
  private final ArmTilt tilt = ArmTilt.getInstance();
  private final Turret turret = Turret.getInstance();

  private ArmPoseCalculator calculator;

  private boolean isInverted = false;

  private Translation3d trans;

  /** Creates a new SetArmPose. */
  public SetArmPose(Translation3d desiredPosition, boolean inverted) {
    isInverted = inverted;

    trans = desiredPosition;

    addRequirements(extend, tilt, turret);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  public SetArmPose(Translation3d desiredPosition) {

    trans = desiredPosition;

    addRequirements(extend, tilt, turret);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    calculator = new ArmPoseCalculator();

    calculator.setTargetTrans(trans);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    tilt.setAngle(calculator.getActualElevationAngleDegrees());
    extend.setPosition(calculator.getExtendDistance());
    turret.angleTurn(calculator.getTurretRotation());
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
