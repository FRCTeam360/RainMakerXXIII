// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.EncoderType;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.hal.PowerDistributionFaults;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANIds;

public class ArmTilt extends SubsystemBase {



  private ArmExtend extend = ArmExtend.getInstance();

  private final CANSparkMax tiltLead = new CANSparkMax(CANIds.TILT_LEAD_ID, MotorType.kBrushless);
  private final CANSparkMax tiltFollow = new CANSparkMax(CANIds.TILT_FOLLOW_ID, MotorType.kBrushless); 
  private final RelativeEncoder encoder;
  
  private final AbsoluteEncoder absoluteEncoder = tiltLead.getAbsoluteEncoder(Type.kDutyCycle);

  private static ArmTilt instance;
  private SparkMaxPIDController pidController;

  private double motorRotationsToArmDegrees = 360.0/(5.0*5.0*4.0*3.0);

  private double kP = 3;
  private double kI = 0;
  private double kD = 0;
  private double kIz = 0;
  private double kFF = 0;
  private double kMaxOutput = 1;
  private double kMinOutput = -1;
  private double maxRPM = 5700;

  // private double maxVel = 2000; //rpm
  // private double minVel = 0;
  // private double maxAcc = 1500;
  // private double allowedErr = 0;

  /** Creates a new Tilt. */
  public ArmTilt() {
    //-38 degrees as the minimum soft limit
    //212 for the upper limit

    tiltLead.restoreFactoryDefaults();
    tiltLead.setInverted(false);
    tiltLead.setIdleMode(IdleMode.kBrake);

    tiltFollow.restoreFactoryDefaults(); 
    tiltFollow.setInverted(false);
    tiltFollow.setIdleMode(IdleMode.kBrake);
    tiltFollow.follow(tiltLead);


    tiltLead.setSoftLimit(SoftLimitDirection.kForward,90.0f);
    tiltLead.setSoftLimit(SoftLimitDirection.kReverse, -30.0f);
    tiltLead.enableSoftLimit(SoftLimitDirection.kForward, true);
    tiltLead.enableSoftLimit(SoftLimitDirection.kReverse, true);

    encoder = tiltLead.getEncoder();

    pidController = tiltLead.getPIDController();

    encoder.setPositionConversionFactor(motorRotationsToArmDegrees);

    pidController.setP(kP);
    pidController.setI(kI);
    pidController.setD(kD);
    pidController.setIZone(kIz);
    pidController.setFF(kFF * (Math.cos(getAngle()) * extend.getDistanceFromPivot()));
    pidController.setOutputRange(kMinOutput, kMaxOutput);

    pidController.setFeedbackDevice(absoluteEncoder);
  }

    public static ArmTilt getInstance() {
    if (instance == null) {
      instance = new ArmTilt();
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
    return tiltLead;
  }
  
  public void adjustTilt(double speed) {
    tiltLead.set(speed);
  }

  public void setAngle(double inputAngle) {
    System.out.println("set angleing " + inputAngle);
    pidController.setReference(inputAngle, ControlType.kPosition);
  }

  public void smartTilt(double inputAngle) {
    pidController.setReference(inputAngle, ControlType.kSmartMotion);
  }

  public double getAngle(){
    return encoder.getPosition();
  }
  public void resetAngle(){
    encoder.setPosition(0.0);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("arm position", encoder.getPosition());
 }
}