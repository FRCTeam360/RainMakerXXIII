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
  private final CANSparkMax motorLead = new CANSparkMax(CANIds.ARM_LEAD_ID, MotorType.kBrushless);
  private final CANSparkMax motorFollow = new CANSparkMax(CANIds.ARM_FOLLOW_ID, MotorType.kBrushless);
  /** Creates a new Arm. */
  public Arm() {
    motorLead.restoreFactoryDefaults();
    motorFollow.restoreFactoryDefaults();

    motorLead.setInverted(false);
    motorFollow.setInverted(false);

    motorLead.setIdleMode(IdleMode.kBrake);
    motorFollow.setIdleMode(IdleMode.kBrake);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
