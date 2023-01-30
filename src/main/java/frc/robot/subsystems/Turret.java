// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.PipedInputStream;
import java.sql.Driver;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANIds;

public class Turret extends SubsystemBase {

  private final CANSparkMax motor;
  private SparkMaxPIDController pidController;
  private RelativeEncoder encoder;

  private static Turret instance;

  //gearBoxRatio through rotataionsPerTick copied from last year for now, need to update
  public static final double gearBoxRatio = 45.0 / 0.0;
  public static final double pulleyRatio = 17.5 / 1.0;
  public static final double degreesPerRotation = 360.0 / 1.0;
  public static final double rotationsPerTick = 1.0 / 42.0;

  /** Creates a new Turret. */
  public Turret() {
    motor = new CANSparkMax(CANIds.TURRET_ID, MotorType.kBrushless);
    motor.restoreFactoryDefaults();
    motor.setInverted(false);
    motor.setIdleMode(IdleMode.kCoast);
    
    pidController = motor.getPIDController();
    encoder = motor.getEncoder();
  }

  public static Turret getInstance() {
    if (instance == null) {
      instance = new Turret();
    }
    return instance;
  }

  public SparkMaxPIDController getPIDController(){
    return pidController;
  }

  public RelativeEncoder getEncoder() {
    return encoder;
  }

  public CANSparkMax getMotor() {
    return motor;
  }

  public void turn(double speed) {
    motor.set(speed);
  }

  public double getAngle() {
    double encoderPosition = motor.getEncoder().getPosition();
    return encoderPosition * gearBoxRatio * pulleyRatio * degreesPerRotation;
  }

  public double getEncoderTick() {
    return motor.getEncoder().getPosition();
  }

  public void angleTurn(double inputAngle) {
    pidController.setReference(inputAngle, ControlType.kPosition);
  }

  public void resetEncoderTicks(){
    motor.getEncoder().setPosition(0);
  }

  public void resetAngle(double inputReset) {
    this.angleTurn(inputReset);
  }

  public void fieldOrientedTurret(double angle) {
    //double relativeAngle = angle - drivetrainAngle; //TODO UPDATE WITH DRIVETRAIN ANGLE !!!
    //PIDControl.setReference(relativeAngle, ControlType.kPosition); 
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
