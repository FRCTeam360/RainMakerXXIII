// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmExtend;
import frc.robot.subsystems.ArmTilt;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Turret;

public class SetPositions extends CommandBase {
  private ArmTilt tilt = ArmTilt.getInstance();
  private ArmExtend extend = ArmExtend.getInstance();
  private Turret turret = Turret.getInstance();

  private double tiltAngle;
  private double extendDistance;
  private double turretAngle;

  private boolean fieldRelTurret;
  private boolean shouldEnd;

  /** Creates a new SetPositions. */
  public SetPositions(double tiltAngle, double extendDistance, double turretAngle, boolean checkAlliance) {
    addRequirements(tilt, extend, turret);
    this.tiltAngle = tiltAngle;
    this.extendDistance = extendDistance;
    this.turretAngle = checkAlliance && DriverStation.getAlliance() == Alliance.Red ? -turretAngle : turretAngle;
    this.fieldRelTurret = false;
    this.shouldEnd = true;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  public SetPositions(double tiltAngle, double extendDistance, double turretAngle) {
    addRequirements(tilt, extend, turret);
    this.tiltAngle = tiltAngle;
    this.extendDistance = extendDistance;
    this.turretAngle = turretAngle;
    this.fieldRelTurret = false;
    this.shouldEnd = true;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  public SetPositions(double tiltAngle, double extendDistance, double turretAngle, boolean checkAlliance, boolean shouldEnd, boolean fieldRelTurret) {
    addRequirements(tilt, extend, turret);
    this.tiltAngle = tiltAngle;
    this.extendDistance = extendDistance;
    this.turretAngle = checkAlliance && DriverStation.getAlliance() == Alliance.Red ? -turretAngle : turretAngle;
    this.fieldRelTurret = fieldRelTurret;
    this.shouldEnd = shouldEnd;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println(this.getClass().getSimpleName() + "started");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // if(Math.abs(extend.getExtendDistance() - extendDistance) <= 0.3){
    tilt.setAngle(tiltAngle);
    // }
    extend.setPosition(extendDistance);
    if(fieldRelTurret){
      turret.fieldOrientedTurret(turretAngle);
    }else{
      turret.angleTurn(turretAngle);
    }
    SmartDashboard.putBoolean("tilt in position", Math.abs(tilt.getAngle() - tiltAngle) < 2);
    SmartDashboard.putBoolean("turret in position",  Math.abs(turret.getAngleRelativeToRobot() - turret.getNearestTurretAngle(turretAngle)) < 1);
    SmartDashboard.putBoolean("extend in position", Math.abs(extend.getExtendDistance() - extendDistance) < 0.02);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println(this.getClass().getSimpleName() + "finished");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return shouldEnd ? Math.abs(tilt.getAngle() - tiltAngle) < 2
        && Math.abs(turret.getAngleRelativeToRobot() - turret.getNearestTurretAngle(turretAngle)) < 1
        && Math.abs(extend.getExtendDistance() - extendDistance) < 0.02
        : false;
  }
}
