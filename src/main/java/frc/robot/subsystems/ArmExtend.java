// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANIds;

public class ArmExtend extends SubsystemBase {
  private static ArmExtend instance;
  private final CANSparkMax motor = new CANSparkMax(CANIds.EXTEND_ID, MotorType.kBrushless);
  private final SparkMaxPIDController pidController;
  private final RelativeEncoder encoder;

  /** Creates a new Extend. */
  public ArmExtend() {
    motor.restoreFactoryDefaults();
    motor.setInverted(false);
    motor.setIdleMode(IdleMode.kBrake);
    
    pidController = motor.getPIDController();
    encoder = motor.getEncoder();
  }

  public static ArmExtend getInstance() {
    if (instance == null) {
      instance = new ArmExtend();
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

  public void adjustExtension(double speed) {
    motor.set(speed); 
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
