// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANIds;

public class ArmExtend extends SubsystemBase {
  private static ArmExtend instance;
  private final CANSparkMax leadMotor = new CANSparkMax(CANIds.EXTEND_LEAD_ID, MotorType.kBrushless);
  private final CANSparkMax followMotor = new CANSparkMax(CANIds.EXTEND_FOLLOW_ID, MotorType.kBrushless);

  private final SparkMaxPIDController pidController;
  private final RelativeEncoder encoder;

  private double rotationsToMeters = (0.0354*(51.875-13.625))/23.8808; //(0.0354*(25.5-4.625))/30.0;

  private double kP = 10;
  private double kI = 0;
  private double kD = 0;
  private double kIz = 0;
  private double kFF = 0;
  private double kMaxOutput = 1;
  private double kMinOutput = -1;
  private double maxRPM = 5700;

  private float forwardLimit = (float)(1.33);
  private float reverseLimit = (float)0.1;

  private double balancePoint = 0.1; //TODO TUNE VALUE
  public double tuningDistance = 0.6; //TODO TUNE VALUE

  ShuffleboardTab tab = Shuffleboard.getTab("Diagnostics");

  /** Creates a new Extend. */
  public ArmExtend() {
    leadMotor.restoreFactoryDefaults();
    leadMotor.setInverted(true);
    leadMotor.setIdleMode(IdleMode.kBrake);

    followMotor.restoreFactoryDefaults();
    followMotor.setInverted(true);
    followMotor.setIdleMode(IdleMode.kBrake);
    followMotor.follow(leadMotor);
    
    pidController = leadMotor.getPIDController();

    pidController.setP(kP);
    pidController.setI(kI);
    pidController.setD(kD);
    pidController.setFF(kFF);
    pidController.setIZone(kIz);
    
    leadMotor.setSoftLimit(SoftLimitDirection.kForward, forwardLimit);
    leadMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
    leadMotor.setSoftLimit(SoftLimitDirection.kReverse, reverseLimit);
    leadMotor.enableSoftLimit(SoftLimitDirection.kReverse, false);


    encoder = leadMotor.getEncoder();

    encoder.setPositionConversionFactor(rotationsToMeters);
    
    tab.addDouble("Arm Extension", () -> encoder.getPosition());
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
    return leadMotor;
  }

  public void adjustExtensionSpeed(double speed) {
    leadMotor.set(speed); 
  }

  public void setPosition(double meters){
    pidController.setReference(meters, ControlType.kPosition);
  }

  public double getExtendDistance(){
    return encoder.getPosition();
  }

  public double getDistanceFromPivot(){
    return getExtendDistance() + 0.0254 * 16.35; //conv factor * inches
  }

  public double getDistanceFromBalance(){
    return getExtendDistance() - balancePoint; 
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    }
}
