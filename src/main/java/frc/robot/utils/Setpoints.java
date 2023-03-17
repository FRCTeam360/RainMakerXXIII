// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.SetClaw;
import frc.robot.commands.SetPositions;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.DriveTrain;

/** Add your docs here. */
public class Setpoints {
    private DriveTrain driveTrain = DriveTrain.getInstance();
    private Claw claw = Claw.getInstance();
    public static Command scoreLeft(){
        return new SetPositions(42, 1.1, 15, false);
    }

    public static Command scoreRight(){
        return new SetPositions(42, 1.1, -15, false);
    }

    public static Command coneSingleStation(){
        System.out.println("CONE SUBSTAION");
        return new ParallelCommandGroup(
            new SetPositions(130, 0, 90, true, true),
            new SetClaw(115)
        );
    }

    public static Command cubeSingleStation(){
        System.out.println("CUBE SUBATATIONS");
        return new ParallelCommandGroup(
            new SetPositions(0, 0, 0, true, true),
            new SetClaw(20)
        );
    }

    // public class SingleStation extends CommandBase{
    //     public SingleStation(){

    //     }

    //     @Override
    //     public void execute() {
    //         if(claw.isConeMode()){
    //             SmartDashboard.putBoolean("isSetpointingCone", true);
    //             return coneSingleStation();
    //         } else {
    //             SmartDashboard.putBoolean("isSetpointingCube", true);
    //             return cubeSingleStation();
    //         }
    //     }
    //     // System.out.println("UR SINGLE");

    //     // return Claw.getInstance().isConeMode() ? coneSingleStation() : cubeSingleStation();
    // }
}
