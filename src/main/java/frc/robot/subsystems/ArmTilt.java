// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.hal.PowerDistributionFaults;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANIds;

public class ArmTilt extends SubsystemBase {

  private final CANSparkMax tiltLead = new CANSparkMax(CANIds.TILT_LEAD_ID, MotorType.kBrushless);
  private final RelativeEncoder encoder;

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
  private double minVel = 0;
  private double maxAcc = 1500;
  private double allowedErr = 0;

  /** Creates a new Tilt. */
  public ArmTilt() {
    
    tiltLead.restoreFactoryDefaults();
    //tiltFollow.restoreFactoryDefaults(); 

    tiltLead.setInverted(false);
    //tiltFollow.setInverted(false);

    tiltLead.setIdleMode(IdleMode.kBrake);
    //tiltFollow.setIdleMode(IdleMode.kBrake);

    encoder = tiltLead.getEncoder();

    pidController = tiltLead.getPIDController();

    pidController.setP(kP);
    pidController.setI(kI);
    pidController.setD(kD);
    pidController.setIZone(kIz);
    pidController.setFF(kFF);
    pidController.setOutputRange(kMinOutput, kMaxOutput);

    int smartMotionSlot = 0;
    pidController.setSmartMotionMaxVelocity(maxVel, smartMotionSlot);
    pidController.setSmartMotionMinOutputVelocity(minVel, smartMotionSlot);
    pidController.setSmartMotionMaxAccel(maxAcc, smartMotionSlot);
    pidController.setSmartMotionAllowedClosedLoopError(allowedErr, smartMotionSlot);

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

    if((p != kP)) { 
      pidController.setP(p); kP = p;}
    if((i != kI)) { 
      pidController.setI(i); kI = i; }
    if((d != kD)) { 
      pidController.setD(d); kD = d; }
    if((iz != kIz)) {
       pidController.setIZone(iz); kIz = iz; }
    if((ff != kFF)) { 
      pidController.setFF(ff); kFF = ff; }
    if((max != kMaxOutput) || (min != kMinOutput)) { 
      pidController.setOutputRange(min, max); 
      kMinOutput = min; kMaxOutput = max; 
    }

    if((maxV != maxVel)) { 
      pidController.setSmartMotionMaxVelocity(maxV,0); maxVel = maxV; }
    if((minV != minVel)) { 
      pidController.setSmartMotionMinOutputVelocity(minV,0); minVel = minV; }
    if((maxA != maxAcc)) { 
      pidController.setSmartMotionMaxAccel(maxA,0); maxAcc = maxA; }
    if((allE != allowedErr)) { 
      pidController.setSmartMotionAllowedClosedLoopError(allE,0); allowedErr = allE; }

    double setPoint, processVariable;

    boolean mode = SmartDashboard.getBoolean("Mode", false);
    if(mode) {
      setPoint = SmartDashboard.getNumber("Set Velocity", 0);
      pidController.setReference(setPoint, CANSparkMax.ControlType.kVelocity);
      processVariable = encoder.getVelocity();
    } else {
      setPoint = SmartDashboard.getNumber("Set Position", 0);
      pidController.setReference(setPoint, CANSparkMax.ControlType.kSmartMotion);
      processVariable = encoder.getPosition();
    }
    
    SmartDashboard.putNumber("SetPoint", setPoint);
    SmartDashboard.putNumber("Process Variable", processVariable);
    SmartDashboard.putNumber("Output", tiltLead.getAppliedOutput());

  }
}