// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class AutoEngage extends CommandBase {
  private final DriveTrain driveTrain = DriveTrain.getInstance();

  private final double kP = 0.0023; //was .003 .002
  private final double kI = 0;
  private final double kD = 0.00;

  private final PIDController pitchController = new PIDController(kP, kI, kD);
  private final PIDController rollController = new PIDController(kP, kI, kD);

  private double pastAngle; 
  /** Creates a new AutoEngage. */
  public AutoEngage() {
    addRequirements(driveTrain);
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
    System.out.println("flattening the covid curve");
    double currentAngle = calculateAngle(driveTrain.getPitch(), driveTrain.getRoll());

    if(currentAngle - pastAngle < -1|| Math.abs(currentAngle) < 2){
      driveTrain.xOut();
    } else {
      driveTrain.drive(
        new ChassisSpeeds(-pitchController.calculate(-driveTrain.getPitch()*driveTrain.getPitch()*Math.signum(driveTrain.getPitch())), rollController.calculate(-driveTrain.getRoll()*driveTrain.getRoll()*Math.signum(driveTrain.getRoll())), 0)
      );
    }

    pastAngle = currentAngle;

    //SmartDashboard.putNumber("error", currentAngle-pastAngle);
    //SmartDashboard.putNumber("current drivetrain angle", currentAngle);
  }

  public double calculateAngle(double pitch, double roll){
    return Math.sqrt(pitch*pitch + roll*roll);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println(this.getClass().getSimpleName() + "finished");
    driveTrain.xOut();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
