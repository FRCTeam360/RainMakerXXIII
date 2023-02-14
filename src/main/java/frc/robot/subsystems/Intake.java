// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANIds;

public class Intake extends SubsystemBase {
  private final CANSparkMax motor = new CANSparkMax(CANIds.INTAKE_ID, MotorType.kBrushless);
  private static Intake instance;
  /** Creates a new Intake. */
  public Intake() {
    motor.restoreFactoryDefaults();
    motor.setInverted(false);
    motor.setIdleMode(IdleMode.kBrake);
    motor.setSmartCurrentLimit(20);
  }

  public static Intake getInstance() {
    if (instance == null) {
      instance = new Intake();
    }

    return instance;
  }

  public void run(double speed) {
    motor.set(speed);
  }

  public void stop(){
    motor.stopMotor();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
