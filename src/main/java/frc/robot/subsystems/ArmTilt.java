// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANIds;

public class ArmTilt extends SubsystemBase {



  private ArmExtend extend = ArmExtend.getInstance();

  private final CANSparkMax tiltLead = new CANSparkMax(CANIds.TILT_LEAD_ID, MotorType.kBrushless);
  private final CANSparkMax tiltFollow = new CANSparkMax(CANIds.TILT_FOLLOW_ID, MotorType.kBrushless); 
  private final RelativeEncoder encoder;
  private final AbsoluteEncoder absoluteEncoder;

  private static ArmTilt instance;
  private SparkMaxPIDController pidController;

  private double motorRotationsToArmDegrees = 360.0/(5.0*5.0*4.0*3.0);
  private double practiceMotorRotationsToArmDegrees = 360.0/(6.0*6.0*(64.0/12.0)); //7, 5

  private double kMaxRampRate = 0.5; //TODO TUNE

  private double kP = 0.02; //3
  private double kI = 0.00000000001;
  private double kD = 0.000005;
  private double kIz = 0;
  public double kFF = 0.045; //0.01 retracted 0.05 extended
  private double kMaxOutput = 0.9;
  private double kMinOutput = -0.9;

  private double kP2 = 0.02; //3
  private double kI2 = 0;
  private double kD2 = 0.5;
  private double kIz2 = 0;
  private double kMaxOutput2 = 0.65;
  private double kMinOutput2 = -0.65;

  private double kP3 = 0.02; //3
  private double kI3 = 0;
  private double kD3 = 0;
  private double kIz3 = 0;
  private double kMaxOutput3 = 0.4;
  private double kMinOutput3 = -0.4;

  // private TrapezoidProfile profile = new TrapezoidProfile(null, null);
  // private double previousSetpoint = 10000;
  // private double maxVel = 10; //degrees per second
  // private double maxAccel = 5; //degress per second squared
  // private Timer timer = new Timer();

  private double maxRPM = 5700;

  private int iterations = 0;

  ShuffleboardTab tab = Shuffleboard.getTab("Diagnostics");

  /** Creates a new Tilt. */
  public ArmTilt() {
    //-38 degrees as the minimum soft limit
    //212 for the upper limit

    tiltLead.restoreFactoryDefaults();
    tiltLead.setInverted(false);
    tiltLead.setIdleMode(IdleMode.kBrake);

    tiltFollow.restoreFactoryDefaults(); 
    tiltFollow.setInverted(true);
    tiltFollow.setIdleMode(IdleMode.kBrake);
    tiltFollow.follow(tiltLead);

    tiltLead.setSoftLimit(SoftLimitDirection.kForward,220.0f + 90f);
    tiltLead.setSoftLimit(SoftLimitDirection.kReverse, -30.0f + 90f);
    tiltLead.enableSoftLimit(SoftLimitDirection.kForward, true);
    tiltLead.enableSoftLimit(SoftLimitDirection.kReverse, true);

    encoder = tiltLead.getEncoder();

    absoluteEncoder = tiltLead.getAbsoluteEncoder(Type.kDutyCycle);
    absoluteEncoder.setPositionConversionFactor(360);
    absoluteEncoder.setZeroOffset(86.5);
    absoluteEncoder.setInverted(true);
    // tiltLead.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 20); //TODO remove if doesn't work

    pidController = tiltLead.getPIDController();

    encoder.setPositionConversionFactor(practiceMotorRotationsToArmDegrees);
    encoder.setVelocityConversionFactor(practiceMotorRotationsToArmDegrees / 60);

    // tiltLead.setClosedLoopRampRate(kMaxRampRate); 

    pidController.setP(kP);
    pidController.setI(kI);
    pidController.setD(kD);
    pidController.setIZone(kIz);
    pidController.setOutputRange(kMinOutput, kMaxOutput);

    pidController.setP(kP2, 1);
    pidController.setI(kI2, 1);
    pidController.setD(kD2, 1);
    pidController.setIZone(kIz2, 1);
    pidController.setOutputRange(kMinOutput2, kMaxOutput2, 1);

    pidController.setP(kP3, 2);
    pidController.setI(kI3, 2);
    pidController.setD(kD3, 2);
    pidController.setIZone(kIz3, 2);
    pidController.setOutputRange(kMinOutput3, kMaxOutput3, 2);

    pidController.setFeedbackDevice(absoluteEncoder);

    tab.addDouble("Arm Tilt", () -> encoder.getPosition());
    tab.addDouble("arm absolute", () -> absoluteEncoder.getPosition() - 90);
    tab.addDouble("tilt ff", () -> kFF);
    tab.addDouble("tilt ff math", () -> kFF * (Math.cos(Math.toRadians(getAngle())) * (extend.getDistanceFromBalance() / extend.maxExtendMinusBalance)));
    tab.addDouble("arm output", () -> tiltLead.getAppliedOutput());
    // SmartDashboard.putNumber("kff", kFF);
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
    tiltLead.set(speed + getFeedForward()/12.0); 
  }

  public void setAngle(double inputAngle) {
    // System.out.println("angle set"+ inputAngle);
    inputAngle = inputAngle + 90;
    if(ArmExtend.getInstance().getExtendDistance() >= 0.8){
      pidController.setReference(inputAngle, ControlType.kPosition, 2, getFeedForward()); //was , getFeedForward()
    } else if(ArmExtend.getInstance().getExtendDistance() >= 0.4){
      pidController.setReference(inputAngle, ControlType.kPosition, 1, getFeedForward());
    } else {
      pidController.setReference(inputAngle, ControlType.kPosition, 0, getFeedForward());
    }
     // SmartDashboard.putNumber("error", getAngle() - inputAngle);
  }

  // public void trapezoidAngle(double inputAngle) {
  //   //generates new profile if desired angle has changed
  //   if(inputAngle != previousSetpoint){
  //     timer.stop();
  //     timer.reset();

  //     //generates a new profile for the new desired angle
  //     profile = new TrapezoidProfile(new Constraints(maxVel, maxAccel), 
  //                                   new State(inputAngle, 0), 
  //                                   new State(getAngle(), getVelocity()));
      
  //     previousSetpoint = inputAngle;
  //     //timer used to determine how far into the profile we are
  //     timer.start();
  //   }

  //   //sets pidController to the setpoint generated by the profile at the timestamp determined by the timer
  //   pidController.setReference(profile.calculate(timer.get()).position, ControlType.kPosition, 0, getFeedForward());
  // }

  // public void altTrapezoidAngle(double inputAngle) {
  //   //creates new profile every 0.02 sec Periodic cycle based off of current position+velocity and desired angle
  //   profile = new TrapezoidProfile(new Constraints(maxVel, maxAccel), 
  //                                 new State(inputAngle, 0), 
  //                                 new State(getAngle(), getVelocity()));

  //   //sets controller to position generated by the profile for the time point of the next Periodic iteration
  //   pidController.setReference(profile.calculate(0.02).position, ControlType.kPosition, 0, getFeedForward());
  // }

  public double getFeedForward() {
    return kFF * 12.0 * (Math.cos(Math.toRadians(getAngle())) * (extend.getDistanceFromBalance() / extend.maxExtendMinusBalance));
  }

  public void smartTilt(double inputAngle) {
    pidController.setReference(inputAngle, ControlType.kSmartMotion);
  }

  public double getAngle(){
    // return encoder.getPosition();
    return absoluteEncoder.getPosition() - 90.0;
  }

  public double getVelocity(){
    return encoder.getVelocity();
  }

  public void resetAngle(){
    encoder.setPosition(0.0);
  }

  public void reseedMotorPosition(){
    if(encoder.getVelocity() < 0.1){
      iterations++;
    } else {
      iterations = 0;
    }

    if(iterations >= 5){
      encoder.setPosition(absoluteEncoder.getPosition() - 90);
      iterations = 0;
      // System.out.println("position reset");
    }
  }

  @Override
  public void periodic() {
    //SmartDashboard.putNumber("arm position", encoder.getPosition());
    //SmartDashboard.putNumber("arm absolute position", absoluteEncoder.getPosition());
    reseedMotorPosition();
    // kFF = SmartDashboard.getNumber("kFF", 0.0);
 }
}