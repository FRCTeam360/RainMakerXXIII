// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.PipedInputStream;
import java.sql.Driver;

import com.ctre.phoenix.sensors.WPI_Pigeon2;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANIds;
import frc.robot.Constants.CANIds.CANivore;

public class Turret extends SubsystemBase {

  private final CANSparkMax motor;
  private SparkMaxPIDController PIDControl;
  private WPI_Pigeon2 gyro = new WPI_Pigeon2(CANivore.DRIVETRAIN_PIGEON_ID, "Swerve");
  
  private static Turret instance;
  private double relativeAngle;

  //gearBoxRatio through rotataionsPerTick copied from last year for now, need to update
  public static final double gearBoxRatio = 45.0 / 0.0;
  public static final double pulleyRatio = 17.5 / 1.0;
  public static final double degreesPerRotation = 360.0 / 1.0;
  public static final double rotationsPerTick = 1.0 / 42.0;

  public static final double conversionFactor = (1.0/20.0) * (1.5/17.5) * (360.0/1.0);
  public static final double impericalConversionFactor = 360.0/542.0;

  public static Turret getInstance() {
    if (instance == null) {
      instance = new Turret();
    }
    return instance;
  }

  /** Creates a new Turret. */
  public Turret() {
    motor = new CANSparkMax(CANIds.TURRET_ID, MotorType.kBrushless);
    motor.restoreFactoryDefaults();
    motor.setInverted(false);
    motor.setIdleMode(IdleMode.kCoast);
    // System.out.println("conv: " + conversionFactor);
    // System.out.println("imp conv: " + impericalConversionFactor);
    motor.getEncoder().setPositionConversionFactor(1/conversionFactor);
    
    PIDControl = motor.getPIDController();
    PIDControl.setP(0.05,1);
    PIDControl.setD(0.01, 1);
    PIDControl.setI(0.0,1);
    PIDControl.setFF(0.0, 1);

  }

  public void turn(double speed) {
    motor.set(speed);
  }

  public double getAngleRelativeToRobot() {
    double encoderPosition = motor.getEncoder().getPosition();
    return encoderPosition;
  }

  public void angleTurn(double inputAngle) {
    PIDControl.setReference(inputAngle, ControlType.kPosition);
  }

  public void resetEncoderTicks(){
    motor.getEncoder().setPosition(0);
  }

  public void resetAngle(double inputReset) {
    this.angleTurn(inputReset);
  }

  public double getRelativeAngle() {
    return relativeAngle;
  }

  public void fieldOrientedTurret(double angle) {
    Rotation2d driveRotation = DriveTrain.getInstance().getGyroscopeRotation();
    double drivetrainAngle = driveRotation.getDegrees();
    relativeAngle = angle - drivetrainAngle; 
    PIDControl.setReference(relativeAngle, ControlType.kPosition,1); 
    // System.out.println("angle: " + angle);
    // System.out.println("relative angle: " + relativeAngle);

  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("turretAngle", motor.getEncoder().getPosition());
    // System.out.println("position: " + motor.getEncoder().getPosition());
    // This method will be called once per scheduler run
  }
}
