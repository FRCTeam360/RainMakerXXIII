// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.hal.PowerDistributionFaults;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANIds;

public class ArmTilt extends SubsystemBase {

  private final CANSparkMax tiltLead = new CANSparkMax(CANIds.TILT_LEAD_ID, MotorType.kBrushless);
  private static ArmTilt instance;
  private SparkMaxPIDController pidController;

  private double kP = 5e-5;
  private double kI = 1e-6;
  private double kD = 0;
  private double kIz = 0;
  private double kFF = 0.000156;
  private double kMaxOutput = 1;
  private double kMinOutput = -1;
  private double maxRPM = 5700;

  private double maxVel = 2000; //rpm
  private double maxAcc = 1500;

  /** Creates a new Tilt. */
  public ArmTilt() {
    
    tiltLead.restoreFactoryDefaults();
    //tiltFollow.restoreFactoryDefaults(); 

    tiltLead.setInverted(false);
    //tiltFollow.setInverted(false);

    tiltLead.setIdleMode(IdleMode.kBrake);
    //tiltFollow.setIdleMode(IdleMode.kBrake);

    pidController = tiltLead.getPIDController();

    pidController.setP(kP);
    pidController.setI(kI);
    pidController.setD(kD);
    pidController.setIZone(kIz);
    pidController.setFF(kFF);
    pidController.setOutputRange(kMinOutput, kMaxOutput);

    int smartMotionSlot = 0;
    pidController.setSmartMotionMaxVelocity(maxVel, smartMotionSlot);
    pidController.setSmartMotionMinOutputVelocity(minVel, smartMotionSlot)
  }

  public static ArmTilt getInstance() {
    if (instance == null) {
      instance = new ArmTilt();
    }

    return instance;
  }
  
  public void adjustTilt(double speed) {
    tiltLead.set(speed);
  }

  public void pidTilt(double inputAngle) {
    SparkMaxPIDController PIDControl = tiltLead.getPIDController();
    PIDControl.setReference(inputAngle, ControlType.kPosition);
  }

  public void smartTilt(double inputAngle) {
    pidController.setReference(inputAngle, ControlType.kSmartMotion);
  }
  
  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
