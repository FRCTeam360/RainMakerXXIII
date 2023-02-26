// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.CANIds;
import frc.robot.Constants.RobotType;

public class Claw extends SubsystemBase {

  private final CANSparkMax motor = new CANSparkMax(CANIds.CLAW_GRIP_ID, MotorType.kBrushless);
  private final RelativeEncoder encoder;
  private final AbsoluteEncoder absoluteEncoder;
  private final SparkMaxPIDController pidController;
  private static Claw instance;

  private int iterations = 0;

  ShuffleboardTab tab = Shuffleboard.getTab("Diagnostics");
  
  private double kP = 0.01;
  
  private boolean isComp;


  /** Creates a new Claw. */
  public Claw() {
    isComp = Constants.getRobotType() == RobotType.COMP;
    
    motor.restoreFactoryDefaults();

    motor.setInverted(!isComp);

    motor.setIdleMode(IdleMode.kBrake);

    motor.setSmartCurrentLimit(20);

    encoder = motor.getEncoder();
    
    // absoluteEncoder = motor.getAbsoluteEncoder(Type.kDutyCycle);
    absoluteEncoder = motor.getAbsoluteEncoder(Type.kDutyCycle); //oops this is extremely bad, dont do this please but we kinda have to
    absoluteEncoder.setPositionConversionFactor(360);
    absoluteEncoder.setZeroOffset(325);
    absoluteEncoder.setInverted(isComp);

    pidController = motor.getPIDController();
    pidController.setP(kP);
    pidController.setFeedbackDevice(absoluteEncoder);

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

  public double getAbsoluteAngle(){
    return absoluteEncoder.getPosition();
  }

  public void setPosition(double angle){
    pidController.setReference(angle, ControlType.kPosition);
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
