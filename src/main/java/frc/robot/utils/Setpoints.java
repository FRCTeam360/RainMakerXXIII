// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.CloseClaw;
import frc.robot.commands.RunIntake;
import frc.robot.commands.RunIntakeReversed;
import frc.robot.commands.SetClaw;
import frc.robot.commands.SetPositions;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.DriveTrain;

/** Add your docs here. */
public class Setpoints {
    private DriveTrain driveTrain = DriveTrain.getInstance();
    private Claw claw = Claw.getInstance();

    public static Command scoreLeftCone(){
        return new SetPositions(42, 1.1, 15, false);
    }

    public static Command scoreRightCone(){
        return new SetPositions(42, 1.1, -15, false);
    }

    public static Command scoreLeftCube(){
        return new SetPositions(35, 0.7, 15, false);
    }

    public static Command scoreRightCube(){
        return new SetPositions(35, 0.7, -15, false);
    }

    public static Command scoreSubCone(){
        return new SetPositions(42, 1.15, -15, true);
    }

    public static Command scoreWallCone(){
        return new SetPositions(42, 1.15, 15, true);
    }

    public static Command scoreSubCube(){
        return new SetPositions(35, 0.7, -15, true);
    }

    public static Command scoreWallCube(){
        return new SetPositions(35, 0.7, 15, true);
    }


    public static Command coneSingleStation(){
        System.out.println("CONE SUBSTAION");
        return new ParallelCommandGroup(
            new SetPositions(130, 0, -90, true, false, true),
            new SetClaw(115),
            new RunIntake()
        );
    }

    public static Command coneSingleTurret(){
        return new SetPositions(90, 0, -90, true, false, true);
    }

    public static Command cubeSingleStation(){
        System.out.println("CUBE SUBATATIONS");
        return new ParallelCommandGroup(
            new SetPositions(17, 0, 90, true, false, true),
            new SetClaw(80), 
            new RunIntake()
        );
    }

    public static Command cubeSingleTurret(){
        return new SetPositions(90, 0, 90, true, false, true);
    }

    public static Command coneDouble(){
        return new ParallelCommandGroup(
            new SetPositions(43, 0.8, 0, false, false, true),
            new SetClaw(15), 
            new RunIntake()
        );
    }

    public static Command groundCube(){
        return new SetPositions(-27, 0.65, 180);
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
