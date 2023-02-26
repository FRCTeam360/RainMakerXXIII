// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.CANIds;
import frc.robot.Constants.RobotType;

public class Intake extends SubsystemBase {
  private final CANSparkMax motor = new CANSparkMax(CANIds.INTAKE_ID, MotorType.kBrushless);
  private static Intake instance;
  private final AbsoluteEncoder absoluteEncoder;

  private boolean isComp;
  /** Creates a new Intake. */
  public Intake() {
    isComp = Constants.getRobotType() == RobotType.COMP;

    motor.restoreFactoryDefaults();
    motor.setInverted(!isComp);
    motor.setIdleMode(IdleMode.kBrake);
    motor.setSmartCurrentLimit(20);

    absoluteEncoder = motor.getAbsoluteEncoder(Type.kDutyCycle);
  }

  public static Intake getInstance() {
    if (instance == null) {
      instance = new Intake();
    }

    return instance;
  }

  public AbsoluteEncoder getClawEncoder(){
    return absoluteEncoder;
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
