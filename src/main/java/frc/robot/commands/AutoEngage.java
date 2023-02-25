// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class AutoEngage extends CommandBase {
  private final DriveTrain driveTrain = DriveTrain.getInstance();

  private final double kP = 0.04;

  private double pastAngle; 
  /** Creates a new AutoEngage. */
  public AutoEngage() {
    addRequirements(driveTrain);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double currentAngle = calculateAngle(driveTrain.getPitch(), driveTrain.getRoll());

    if(currentAngle - pastAngle < -0.7|| Math.abs(currentAngle) < 2){
      driveTrain.xOut();
    } else {
      driveTrain.drive(
        new ChassisSpeeds(-driveTrain.getPitch() * kP, driveTrain.getRoll() * kP, 0)
      );
    }

    pastAngle = currentAngle;

    SmartDashboard.putNumber("error", currentAngle-pastAngle);
    SmartDashboard.putNumber("current drivetrain angle", currentAngle);
  }

  public double calculateAngle(double pitch, double roll){
    return Math.sqrt(pitch*pitch + roll*roll);
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
