// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class GroundPickup extends ParallelCommandGroup {
  /** 
   * makes arm go to ground and open claw
   * @param isCone true if picking up cone, false if picking up cube
   */
  public GroundPickup(boolean isCone) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new SetArmPose(new Translation3d(0.3, 0, 0.05), false), 
      isCone ? new OpenClawConeGround() : new OpenClawCubeGround() 
    );
  }
}
