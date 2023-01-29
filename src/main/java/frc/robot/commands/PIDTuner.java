// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.ArmExtend;
import frc.robot.subsystems.ArmTilt;

import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

public class PIDTuner extends CommandBase {

  private double kP = 5e-5;
  private double kI = 1e-6;
  private double kD = 0;
  private double kIz = 0;
  private double kFF = 0.000156;
  private DoubleSupplier feedForwardMultiplier;
  private double kMaxOutput = 1;
  private double kMinOutput = -1;
  private double maxRPM = 5700;

  private double maxVel = 2000; // rpm
  private double minVel = 0;
  private double maxAcc = 1500;
  private double allowedErr = 0;
  private double setPosition = 0.0;

  private static RelativeEncoder encoder;
  private static CANSparkMax motor;
  private static SparkMaxPIDController pidController;

  /** Creates a new PIDTuner. */
  public PIDTuner(SubsystemBase subsystem, RelativeEncoder subsystemEncoder, CANSparkMax subsystemMotor,
      SparkMaxPIDController subsystemPidController, DoubleSupplier feedForwardMultiplier) {
    // Use addRequirements() here to declare subsystem dependencies.

    addRequirements(subsystem);

    encoder = subsystemEncoder;
    motor = subsystemMotor;
    pidController = subsystemPidController;
    this.feedForwardMultiplier = feedForwardMultiplier;

    pidController.setP(kP);
    pidController.setI(kI);
    pidController.setD(kD);
    pidController.setIZone(kIz);
    pidController.setFF(kFF * this.feedForwardMultiplier.getAsDouble());
    // pidController.setFF(kFF * (Math.cos(getAngle()) *
    // extend.getDistanceFromPivot()));
    pidController.setOutputRange(kMinOutput, kMaxOutput);

    int smartMotionSlot = 0;
    pidController.setSmartMotionMaxVelocity(maxVel, smartMotionSlot);
    pidController.setSmartMotionMinOutputVelocity(minVel, smartMotionSlot);
    pidController.setSmartMotionMaxAccel(maxAcc, smartMotionSlot);
    pidController.setSmartMotionAllowedClosedLoopError(allowedErr, smartMotionSlot);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SmartDashboard.putNumber("P Gain", kP);
    SmartDashboard.putNumber("I Gain", kI);
    SmartDashboard.putNumber("D Gain", kD);
    SmartDashboard.putNumber("I Zone", kIz);
    SmartDashboard.putNumber("Feed Forward", kFF);
    SmartDashboard.putNumber("Max Output", kMaxOutput);
    SmartDashboard.putNumber("Min Output", kMinOutput);

    SmartDashboard.putNumber("Max Velocity", maxVel);
    SmartDashboard.putNumber("Min Velocity", minVel);
    SmartDashboard.putNumber("Max Acceleration", maxAcc);
    SmartDashboard.putNumber("Allowed Closed Loop Error", allowedErr);
    SmartDashboard.putNumber("Set Position", 0);
    SmartDashboard.putNumber("Set Velocity", 0);
    
    SmartDashboard.putBoolean("Mode", true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putNumber("Feed Forward Multiplier", feedForwardMultiplier.getAsDouble());

    double p = SmartDashboard.getNumber("P Gain", 0);
    double i = SmartDashboard.getNumber("I Gain", 0);
    double d = SmartDashboard.getNumber("D Gain", 0);
    double iz = SmartDashboard.getNumber("I Zone", 0);
    double ff = SmartDashboard.getNumber("Feed Forward", 0);
    double max = SmartDashboard.getNumber("Max Output", 0);
    double min = SmartDashboard.getNumber("Min Output", 0);
    double maxV = SmartDashboard.getNumber("Max Velocity", 0);
    double minV = SmartDashboard.getNumber("Min Velocity", 0);
    double maxA = SmartDashboard.getNumber("Max Acceleration", 0);
    double allE = SmartDashboard.getNumber("Allowed Closed Loop Error", 0);

    if ((p != kP)) {
      pidController.setP(p);
      kP = p;
    }
    if ((i != kI)) {
      pidController.setI(i);
      kI = i;
    }
    if ((d != kD)) {
      pidController.setD(d);
      kD = d;
    }
    if ((iz != kIz)) {
      pidController.setIZone(iz);
      kIz = iz;
    }
    if ((ff != kFF)) {
      pidController.setFF(ff);
      kFF = ff * feedForwardMultiplier.getAsDouble();
    }
    if ((max != kMaxOutput) || (min != kMinOutput)) {
      pidController.setOutputRange(min, max);
      kMinOutput = min;
      kMaxOutput = max;
    }

    if ((maxV != maxVel)) {
      pidController.setSmartMotionMaxVelocity(maxV, 0);
      maxVel = maxV;
    }
    if ((minV != minVel)) {
      pidController.setSmartMotionMinOutputVelocity(minV, 0);
      minVel = minV;
    }
    if ((maxA != maxAcc)) {
      pidController.setSmartMotionMaxAccel(maxA, 0);
      maxAcc = maxA;
    }
    if ((allE != allowedErr)) {
      pidController.setSmartMotionAllowedClosedLoopError(allE, 0);
      allowedErr = allE;
    }
    double setPoint, processVariable;

    // boolean mode = SmartDashboard.getBoolean("Mode", false);
    // if(mode) {
    // setPoint = SmartDashboard.getNumber("Set Velocity", 0);
    // pidController.setReference(setPoint, CANSparkMax.ControlType.kVelocity);
    // processVariable = encoder.getVelocity();
    // } else {
    // setPoint = SmartDashboard.getNumber("Set Position", 0);
    // pidController.setReference(setPoint, CANSparkMax.ControlType.kSmartMotion);
    // processVariable = encoder.getPosition();
    // }

    setPoint = SmartDashboard.getNumber("Set Position", 0);
    if (setPoint != setPosition){
      pidController.setReference(setPoint, CANSparkMax.ControlType.kPosition);
      setPosition = setPoint;
    }
    processVariable = encoder.getPosition();

    SmartDashboard.putNumber("SetPoint", setPoint);
    SmartDashboard.putNumber("Process Variable", processVariable);
    SmartDashboard.putNumber("Output", motor.getAppliedOutput());
    SmartDashboard.putNumber("Position", encoder.getPosition());

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
