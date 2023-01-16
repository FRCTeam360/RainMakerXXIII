// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANIds;

public class ArmExtend extends SubsystemBase {
  private final CANSparkMax extendMotor = new CANSparkMax(CANIds.EXTEND_ID, MotorType.kBrushless);
  private static ArmExtend instance;

  /** Creates a new Extend. */
  public ArmExtend() {
    extendMotor.restoreFactoryDefaults();
    extendMotor.setInverted(false);
    extendMotor.setIdleMode(IdleMode.kBrake);
    
  }

  public static ArmExtend getInstance() {
    if (instance == null) {
      instance = new ArmExtend();
    }

    return instance;
  }

  public void adjustExtension(double speed) {
    extendMotor.set(speed); 
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
