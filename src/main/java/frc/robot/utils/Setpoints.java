// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.SetClaw;
import frc.robot.commands.SetPositions;

/** Add your docs here. */
public class Setpoints {
    public static Command scoreLeft(){
        return new SetPositions(42, 1.1, 15, false);
    }

    public static Command scoreRight(){
        return new SetPositions(42, 1.1, -15, false);
    }

    public static Command coneSingle(){
        return new ParallelCommandGroup(
            new SetPositions(130, 0, 0), 
            new SetClaw(115)
        );
    }

    public static Command cubeSingle(){
        return new ParallelCommandGroup(
            new SetPositions(0, 0, 0),
            new SetClaw(0)
        );
    }
}
