// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.ColorFlowAnimation;
import com.ctre.phoenix.led.FireAnimation;
import com.ctre.phoenix.led.SingleFadeAnimation;
import com.ctre.phoenix.led.TwinkleAnimation;
import com.ctre.phoenix.led.CANdle.LEDStripType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANIds;
import frc.robot.Constants.SwerveConstants;

public class Lights extends SubsystemBase {

  private static Lights instance;

  private final CANdle candle = new CANdle(CANIds.CANDLE_ID);

  /** Creates a new Lights. */
  public Lights() {
    candle.configLEDType(LEDStripType.GRB);
  }

  public static Lights getInstance() {
    if (instance == null) {
      instance = new Lights();
    }

    return instance;
  }

  public void setIndividualRed(int index){
    candle.setLEDs(254, 0, 0, 0, index, 1);
  }

  public void setINdividualGreen(int index){
    candle.setLEDs(0, 254, 0, 0, index, 1);
  }

  public void setYellow(){
    candle.setLEDs(239, 223, 13);
  }

  public void setPurple(){
    candle.setLEDs(163, 23, 172);
  }

  public void setOrange(){
    candle.setLEDs(254, 106, 0);
  }

  public void setBlue(){
    candle.setLEDs(0, 40, 95);
  }

  public void setBlueTwinkle(){
    candle.animate(new TwinkleAnimation(0, 40, 95));
  }

  public void setOrangeFade(){
    candle.animate(new SingleFadeAnimation(254, 106, 0));
  }

  public void setRedTwinkle(){
    candle.animate(new TwinkleAnimation(255, 0, 0));
  }

  public void fireball(){
    candle.animate(new FireAnimation());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
