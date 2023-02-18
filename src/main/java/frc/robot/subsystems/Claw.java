// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANIds;

public class Claw extends SubsystemBase {

  private final CANSparkMax motor = new CANSparkMax(CANIds.CLAW_GRIP_ID, MotorType.kBrushless);
  private final RelativeEncoder encoder;
  private final AbsoluteEncoder absoluteEncoder;
  private static Claw instance;

  private int iterations = 0;

  ShuffleboardTab tab = Shuffleboard.getTab("Diagnostics");


  /** Creates a new Claw. */
  public Claw() {
    motor.restoreFactoryDefaults();

    motor.setInverted(false);

    motor.setIdleMode(IdleMode.kBrake);

    motor.setSmartCurrentLimit(20);

    encoder = motor.getEncoder();
    
    // absoluteEncoder = motor.getAbsoluteEncoder(Type.kDutyCycle);
    absoluteEncoder = Intake.getInstance().getClawEncoder(); //oops this is extremely bad, dont do this please but we kinda have to
    absoluteEncoder.setPositionConversionFactor(360);
    absoluteEncoder.setZeroOffset(86.5);
    absoluteEncoder.setInverted(true);

    tab.addDouble("Claw position", () -> encoder.getPosition());
    tab.addDouble("Claw absolute", () -> absoluteEncoder.getPosition());

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

  public double getCurrent(){
    return motor.getOutputCurrent();
  }

    public void reseedMotorPosition(){
    if(encoder.getVelocity() < 0.1){
      iterations++;
    } else {
      iterations = 0;
    }

    if(iterations >= 100){
      encoder.setPosition(absoluteEncoder.getPosition() - 90);
      iterations = 0;
      System.out.println("position reset");
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
