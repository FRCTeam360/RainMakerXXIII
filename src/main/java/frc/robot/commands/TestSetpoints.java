// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmExtend;
import frc.robot.subsystems.ArmTilt;
import frc.robot.subsystems.Turret;

public class TestSetpoints extends CommandBase {
  private static ArmExtend extend = ArmExtend.getInstance();
  private static ArmTilt tilt = ArmTilt.getInstance();
  private static Turret turret = Turret.getInstance();
  /** Creates a new TestSetpoints. */
  public TestSetpoints() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SmartDashboard.putNumber("Extend Setpoint", 0);
    SmartDashboard.putNumber("Tilt Setpoint", 0);
    SmartDashboard.putNumber("Turret Setpoint", 0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    extend.setPosition(SmartDashboard.getNumber("Extend Setpoint", 0));
    tilt.setPosition(SmartDashboard.getNumber("Tilt Setpoint", 0));
    turret.angleTurn(SmartDashboard.getNumber("Turret Setpoint", 0));
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
