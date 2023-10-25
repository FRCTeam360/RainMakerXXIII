// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.ColorFlowAnimation;
import com.ctre.phoenix.led.FireAnimation;
import com.ctre.phoenix.led.LarsonAnimation;
import com.ctre.phoenix.led.SingleFadeAnimation;
import com.ctre.phoenix.led.TwinkleAnimation;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.ColorFlowAnimation.Direction;
import com.ctre.phoenix.led.LarsonAnimation.BounceMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANIds;
import frc.robot.Constants.SwerveConstants;

public class Lights extends SubsystemBase {

  private static Lights instance;

  private final CANdle candle = new CANdle(CANIds.CANDLE_ID);

  private boolean shouldShowStatus = true;

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
    if(shouldShowStatus){
      candle.setLEDs(0,0,0,0,0,1);
      candle.setLEDs(0,0,0,0,3,2);
      candle.setLEDs(0,0,0,0,7,1);
      candle.setLEDs(254, 0, 0, 0, index, 1);
    }
  }

  public void setINdividualGreen(int index){
    if(shouldShowStatus){
      candle.setLEDs(0,0,0,0,0,1);
      candle.setLEDs(0,0,0,0,3,2);
      candle.setLEDs(0,0,0,0,7,1);
      candle.setLEDs(0, 254, 0, 0, index, 1);
    }
  }

  public void setYellow(){
    shouldShowStatus = false;
    candle.clearAnimation(0);
    candle.clearAnimation(1);
    candle.setLEDs(255, 200, 0); //239, 223, 13
  }

  public void setPurple(){
    shouldShowStatus = false;
    candle.clearAnimation(0);
    candle.clearAnimation(1);
    candle.setLEDs(163, 23, 172);
  }

  public void setShouldShowStatus() {
    shouldShowStatus = true;
  }

  public void setOrange(){
    candle.setLEDs(250, 40, 0);
  }

  public void setBlue(){
    candle.setLEDs(0, 0, 255);
  }

  public void setBlueTwinkle(){
    candle.animate(new TwinkleAnimation(0, 0, 255));
  }

  public void setBlueFade(){
    candle.clearAnimation(0);
    candle.clearAnimation(1);
    candle.animate(new SingleFadeAnimation(0, 0, 255, 0, 0.1, 100, 9));
  }

  public void setOrangeFade(){
    candle.animate(new SingleFadeAnimation(250, 40, 0, 0, 0.1, 100, 9));
    // System.out.println("setting orange");
  }

  public void setOrangeLarson(){
    Animation orange = new LarsonAnimation(250, 40, 0, 0, 0.1, 48, BounceMode.Front , 7, 8);
    Animation blue = new LarsonAnimation(0, 0, 255, 0, 0.1, 48, BounceMode.Front , 7, 56);

    // candle.animate(new LarsonAnimation(250, 40, 0, 0, 0.1, 48, BounceMode.Front , 7, 8));
    // candle.animate(new LarsonAnimation(0, 40, 95, 0, 0.1, 48, BounceMode.Front , 7, 56));

    candle.animate(orange, 0);
    candle.animate(blue, 1);
  }

  public void setRedTwinkle(){
    candle.animate(new TwinkleAnimation(255, 0, 0));
  }

  public void setRedFade(){
    candle.clearAnimation(0);
    candle.clearAnimation(1);
    candle.animate(new SingleFadeAnimation(255, 0, 0, 0, 0.1, 100, 9));
  }

  public void setRed(){
    candle.setLEDs(255, 0, 0);
  }

  public void fireball(){
    candle.animate(new FireAnimation());
  } 

  public void engaged(){
    Animation orange = new ColorFlowAnimation(250, 40, 0, 0, 0.1, 48, Direction.Forward, 8);
    Animation blue = new ColorFlowAnimation(0, 0, 255, 0, 0.1, 48, Direction.Forward, 56);

    // candle.animate(new LarsonAnimation(250, 40, 0, 0, 0.1, 48, BounceMode.Front , 7, 8));
    // candle.animate(new LarsonAnimation(0, 40, 95, 0, 0.1, 48, BounceMode.Front , 7, 56));

    candle.animate(orange, 0);
    candle.animate(blue, 1);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
