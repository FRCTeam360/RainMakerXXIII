// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANIds;

public class Turret extends SubsystemBase {
  private final CANSparkMax motor;
  
  private static Turret instance;
  //gearBoxRatio through rotataionsPerTick copied from last year for now, need to update
  public static final double gearBoxRatio = 1;
  public static final double pulleyRatio = 1;
  public static final double degreesPerRotation = 1;
  public static final double rotationsPerTick = 1;

  public static double leftSoftLimit;
  public static double rightSoftLimit;
  public static float leftSoftLimitEncoder = (float) (leftSoftLimit / gearBoxRatio / pulleyRatio / degreesPerRotation);
  public static float rightSoftLimitEncoder = (float) (rightSoftLimit / gearBoxRatio / pulleyRatio / degreesPerRotation);

  public static Turret getInstance() {
    if (instance == null) {
      instance = new Turret();
    }
    return instance;
  }

  /** Creates a new Turret. */
  public Turret() {
    motor = new CANSparkMax(CANIds.TURRET_ID, MotorType.kBrushless);
    motor.restoreFactoryDefaults();
    motor.setInverted(false);
    motor.setIdleMode(IdleMode.kCoast);

    motor.setSoftLimit(SoftLimitDirection.kForward, leftSoftLimitEncoder);
    motor.setSoftLimit(SoftLimitDirection.kReverse, rightSoftLimitEncoder);

    motor.enableSoftLimit(SoftLimitDirection.kForward, false);
    motor.enableSoftLimit(SoftLimitDirection.kReverse, false);
  }

  public double getAngle() {
    double encoderPosition = motor.getEncoder().getPosition();
    return encoderPosition * gearBoxRatio * pulleyRatio * degreesPerRotation;
  }

  public void angleTurn(double inputAngle) {
    SparkMaxPIDController PIDControl = motor.getPIDController();
    PIDControl.setReference(inputAngle, ControlType.kPosition);
  }

  public void turn(double speed) {
    motor.set(speed);
  }

  public double getEncoderTick() {
    return motor.getEncoder().getPosition();
  }

  public void resetEncoderTicks(){
    motor.getEncoder().setPosition(0);
  }

  public void resetAngle(double inputReset) {
    this.angleTurn(inputReset);
  }

  public boolean atLeftLimit() {
    return this.getAngle() >= leftSoftLimit && motor.getEncoder().getVelocity() > 0;
  }

  public boolean atRightLimit() {
    return this.getAngle() <= rightSoftLimit && motor.getEncoder().getVelocity() < 0;
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
