// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANIds;

public class Claw extends SubsystemBase {

  private final CANSparkMax motor = new CANSparkMax(CANIds.CLAW_ID, MotorType.kBrushless);
  private static Claw instance;
  /** Creates a new Claw. */
  public Claw() {
    motor.restoreFactoryDefaults();

    motor.setInverted(false);

    motor.setIdleMode(IdleMode.kBrake);

    motor.setSmartCurrentLimit(20);

  }

  public static Claw getInstance() {
    if (instance == null) {
      instance = new Claw();
    }

    return instance;
  }

  public void adjustsClaw(double speed){
    motor.set(speed);
  }

  public void stopClaw(){
    motor.stopMotor();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
