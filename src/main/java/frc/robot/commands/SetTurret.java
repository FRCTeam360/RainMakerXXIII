// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Turret;

public class SetTurret extends CommandBase {
  private Turret turret = Turret.getInstance();

  private double angle;
  private boolean shouldEnd;
  
  /** Creates a new SetTurret. */
  public SetTurret(double angle, boolean shouldEnd, boolean checkAlliance){
    this(checkAlliance && DriverStation.getAlliance() == Alliance.Red ? -angle : angle, 
        shouldEnd);
  }

  public SetTurret(double angle, boolean shouldEnd) {
    this.angle = angle;
    this.shouldEnd = shouldEnd;
    addRequirements(turret);
  }

  public SetTurret(double angle){
    this(angle, false);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println(this.getClass().getSimpleName() + "started");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    turret.setPosition(angle);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println(this.getClass().getSimpleName() + "finished");
    turret.turn(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return shouldEnd ? Math.abs(turret.getAngleRelativeToRobot() - turret.getNearestTurretAngle(angle)) < 1 : false;
  }
}
