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

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.CANIds;
import frc.robot.Constants.DigitalIOIds;
import frc.robot.Constants.RobotType;

public class Turret extends SubsystemBase {

  private final CANSparkMax motor;
  private SparkMaxPIDController pidController;
  private RelativeEncoder encoder;

  private final DigitalInput limitSwitch = new DigitalInput(DigitalIOIds.TURRET_LIMIT_SWITCH_ID);
  private boolean pastLimitSwitchState = false;

  private static Turret instance;
  private double relativeAngle;

  public static final double conversionFactorWoodBot = 1 / ((1.0 / 20.0) * (1.5 / 17.5) * (360.0 / 1.0)); //gearbox pulley teeth degrees
  public static final double conversionFactorPractice = (1.0 / 16.0) * (24.0 / 300.0) * (360.0 / 1.0); //same as comp

  public static final float softLimitForwardPractice = 270.0f;
  public static final float softLimitReversePractice = -180.0f;

  public static final float softLimitForwardComp = 270.0f;
  public static final float softLimitReverseComp = -270.0f;

  public static float softLimitForward;
  public static float softLimitReverse;

  ShuffleboardTab tab = Shuffleboard.getTab("Diagnostics");

  /** Creates a new Turret. */
  public Turret() {
    motor = new CANSparkMax(CANIds.TURRET_ID, MotorType.kBrushless);
    motor.restoreFactoryDefaults();
    motor.setInverted(false);
    motor.setIdleMode(IdleMode.kBrake);

    softLimitForward = Constants.getRobotType() == RobotType.COMP ? softLimitForwardComp : softLimitForwardPractice;
    softLimitReverse = Constants.getRobotType() == RobotType.COMP ? softLimitReverseComp : softLimitReversePractice;

    motor.getEncoder().setPositionConversionFactor(Constants.getRobotType() == RobotType.PRACTICE ? conversionFactorPractice : 
        Constants.getRobotType() == RobotType.DRAFT ? conversionFactorWoodBot : conversionFactorPractice); // delete last if and comp factor if no difference from practice

    motor.setSoftLimit(SoftLimitDirection.kForward, softLimitForward);
    motor.setSoftLimit(SoftLimitDirection.kReverse, softLimitReverse);
    motor.enableSoftLimit(SoftLimitDirection.kForward, true);
    motor.enableSoftLimit(SoftLimitDirection.kReverse, true);

    pidController = motor.getPIDController();
    pidController.setP(0.03, 0);
    pidController.setD(0.00, 0); 
    pidController.setI(0.0, 0);
    pidController.setFF(0.0, 0);
    pidController.setOutputRange(-0.9, 0.9, 0); //TODO TUNE

    pidController.setP(0.03, 1);
    pidController.setOutputRange(-0.5, 0.5, 1); //TODO TUNE

    pidController.setP(0.03, 2);
    pidController.setOutputRange(-0.3, 0.3, 2); //TODO TUNE
    encoder = motor.getEncoder();

    tab.addDouble("Turret Angle", () -> encoder.getPosition());
  }

  public static Turret getInstance() {
    if (instance == null) {
      instance = new Turret();
    }
    return instance;
  }

  public SparkMaxPIDController getPIDController() {
    return pidController;
  }

  public RelativeEncoder getEncoder() {
    return encoder;
  }

  public CANSparkMax getMotor() {
    return motor;
  }

  public void turn(double speed) {
    motor.set(speed);
  }

  public double getAngleRelativeToRobot() {
    double encoderPosition = motor.getEncoder().getPosition();
    return encoderPosition;
  }

  public double getEncoderTick() {
    return motor.getEncoder().getPosition();
  }

  public void angleTurn(double inputAngle) {
    setPosition(getNearestActualTurretAngle(inputAngle));
  }

  public void setPosition(double inputAngle) {
    double distance = 0;// ArmExtend.getInstance().getExtendDistance() * Math.abs(Math.cos(ArmTilt.getInstance().getAngle()));
    if(distance > 0.8){
      pidController.setReference(inputAngle, ControlType.kPosition, 2);
    } else if (distance > 0.4){
      pidController.setReference(inputAngle, ControlType.kPosition, 1);
    } else {
      pidController.setReference(inputAngle, ControlType.kPosition, 0);
    }
  }

  public void stop() {
    motor.stopMotor();

  }

  public void resetEncoderTicks() {
    motor.getEncoder().setPosition(0);
  }

  public void resetAngle(double inputReset) {
    this.getEncoder().setPosition(inputReset);
  }

  public double getRelativeAngle() {
    return relativeAngle;
  }

  public void fieldOrientedTurret(double angle) {
    Rotation2d driveRotation = DriveTrain.getInstance().getGyroscopeRotation();
    double drivetrainAngle = driveRotation.getDegrees();
    relativeAngle = angle - drivetrainAngle;
    pidController.setReference(relativeAngle, ControlType.kPosition, 1);
  }

  public double getNearestTurretAngle(double angle){
    double turretFactor = (double) Math.round((getAngleRelativeToRobot() - angle) / 360.0);
    return angle + (360.0 * turretFactor);
  }

  //accounts for soft limits by wraping impossible angles by 90 degrees
  private double getNearestActualTurretAngle(double angle){
    angle = getNearestTurretAngle(angle);

    if(angle > softLimitForward){ 
      return getNearestActualTurretAngle(angle - 360.0);
    } else if(getNearestTurretAngle(angle) < softLimitReverse){
      return getNearestActualTurretAngle(angle + 360.0);
    } else {
      return getNearestTurretAngle(angle);
    }
  }

  private double getNearestLimitSwitchPosition(){
    double turretFactor = (double) Math.round((getAngleRelativeToRobot()) / 180.0);
    return (180.0 * turretFactor);
  }

  private void checkLimitSwitch(){
    boolean currentLimitState = limitSwitch.get();

    if (currentLimitState == false && pastLimitSwitchState == true) {
        resetAngle(getNearestLimitSwitchPosition());
        System.out.println("lil zero");
    }

    pastLimitSwitchState = currentLimitState;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("turret angle", getAngleRelativeToRobot());
    SmartDashboard.putNumber("relative position", getNearestTurretAngle(40.5));
    // tab.addNumber("Turret Angle", () -> motor.getEncoder().getPosition());
    // This method will be called once per scheduler run
    checkLimitSwitch();
  }
}
