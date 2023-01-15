// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANIds;

public class Arm extends SubsystemBase {

  private final CANSparkMax extendMotor = new CANSparkMax(CANIds.EXTEND_ID, MotorType.kBrushless);
  private final CANSparkMax tiltLead = new CANSparkMax(CANIds.TILT_LEAD_ID, MotorType.kBrushless);

  private final double gearBoxRatio = 300;
  //private final CANSparkMax tiltFollow = new CANSparkMax(CANIds.TILT_FOLLOW_ID, MotorType.kBrushless);

  private static Arm instance;
  
  /** Creates a new Arm. */
  public Arm() {
    extendMotor.restoreFactoryDefaults();
    extendMotor.setInverted(false);
    extendMotor.setIdleMode(IdleMode.kBrake);

    tiltLead.restoreFactoryDefaults();
    //tiltFollow.restoreFactoryDefaults(); 

    tiltLead.setInverted(false);
    //tiltFollow.setInverted(false);

    tiltLead.setIdleMode(IdleMode.kBrake);
    //tiltFollow.setIdleMode(IdleMode.kBrake);
  }

  public static Arm getInstance() {
    if (instance == null) {
      instance = new Arm();
    }

    return instance;
  }

  public void adjustExtension(double speed) {
    extendMotor.set(speed); //TODO: NOT REAL NUMBER !! WORKING!!!
  }

  public void adjustTilt(double speed) {
    tiltLead.set(speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
