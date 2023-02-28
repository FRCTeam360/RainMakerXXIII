// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Turret;

public class SetTurretAngle extends CommandBase {
  private final Turret turret = Turret.getInstance();

  private double turretAngle;

  /** Creates a new Homing. */
  public SetTurretAngle(double turretAngle) {
    this.turretAngle = turretAngle;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(turret);
  }

  /** Creates a new Homing. */
  public SetTurretAngle(double turretAngle, boolean checkAlliance) {
    this.turretAngle = checkAlliance && DriverStation.getAlliance() == Alliance.Red ? -turretAngle : turretAngle;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(turret);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println(this.getClass().getSimpleName() + " started");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    turret.setPosition(turretAngle);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println(this.getClass().getSimpleName() + " finished");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(turret.getAngleRelativeToRobot() - 0) < 2;
  }
}
