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

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

  public enum GamePiece {
    CONE, CUBE, NONE
  };

  private GamePiece gamePiece = GamePiece.CUBE;

  private Lights lights = Lights.getInstance();

  private XboxController driverCont = new XboxController(0);
  private XboxController operatorCont = new XboxController(1);

  private int iterations = 0;

  ShuffleboardTab tab = Shuffleboard.getTab("Diagnostics");

  private double kP = 0.01;
  private double kI = 0.000001;
  private double kD = 0.01;
  private double kIZone = 1.0;
  private double kFF = 0;

  private boolean isComp;

  /** Creates a new Claw. */
  public Claw() {
    isComp = Constants.getRobotType() == RobotType.COMP;

    // gamePiece = GamePiece.NONE;

    motor.restoreFactoryDefaults();

    motor.setInverted(!isComp);

    motor.setIdleMode(IdleMode.kBrake);

    motor.setSmartCurrentLimit(20);

    encoder = motor.getEncoder();

    // absoluteEncoder = motor.getAbsoluteEncoder(Type.kDutyCycle);
    absoluteEncoder = motor.getAbsoluteEncoder(Type.kDutyCycle); // oops this is extremely bad, dont do this please but
                                                                 // we kinda have to
    absoluteEncoder.setPositionConversionFactor(360);
    absoluteEncoder.setInverted(isComp);
    absoluteEncoder.setZeroOffset(Constants.getRobotType() == RobotType.COMP ? 95 : 75); // 325 COMP WAS 35, COMP worked
                                                                                         // at 103.2 for pid tuning

    pidController = motor.getPIDController();
    pidController.setP(kP);
    pidController.setI(kI);
    pidController.setD(kD);
    pidController.setIZone(kIZone);
    pidController.setFF(kFF);
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

  public void adjustsClaw(double speed) {
    motor.set(speed);
  }

  public void stopClaw() {
    motor.stopMotor();
  }

  public double getCurrent() {
    return motor.getOutputCurrent();
  }

  public double getAbsoluteAngle() {
    return absoluteEncoder.getPosition();
  }

  public void setPosition(double angle) {
    pidController.setReference(angle, ControlType.kPosition);
  }

  public void reseedMotorPosition() {
    if (encoder.getVelocity() < 0.1) {
      iterations++;
    } else {
      iterations = 0;
    }

    if (iterations >= 100) {
      encoder.setPosition(absoluteEncoder.getPosition() - 90);
      iterations = 0;
    }
  }

  private void checkGamePieceMode() {
    if (driverCont.getBackButton() || operatorCont.getBackButton()) {
      gamePiece = GamePiece.CONE;

      lights.setYellow();
    } else if (driverCont.getStartButton() || operatorCont.getStartButton()) {
      gamePiece = GamePiece.CUBE;
      lights.setPurple();
    } else {
      // lights.setShouldShowStatus();
    }
  }

  public boolean isConeMode() {
    return gamePiece == GamePiece.CONE;
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Mode", isConeMode());
    checkGamePieceMode();
    if (DriverStation.isDisabled()) {
    }
    // This method will be called once per scheduler run
  }
}
