// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.commands.RunIntake;
import frc.robot.commands.SetClaw;
import frc.robot.commands.SetExtend;
import frc.robot.commands.SetPositions;
import frc.robot.commands.SetTilt;

/** Add your docs here. */
public class Setpoints {

    public static boolean isRedAlliance() {
        return DriverStation.getAlliance() == Alliance.Red;
    }

    //TELE OP
    public static Command scoreLeftCone(){
        return new SetPositions(42, 1.1, 15, false);
    }

    public static Command scoreRightCone(){
        return new SetPositions(42, 1.1, -15, false);
    }

    public static Command scoreLeftCube(){
        return new SetPositions(30, 0.7, 9, false);
    }

    public static Command scoreRightCube(){
        return new SetPositions(33, 0.7, -9, false);
    }

    public static Command scoreMidConeLeft() {
        return new SetPositions(45, 0.4, -20, false);
    }

    public static Command scoreMidConeRight() {
        return new SetPositions(45, 0.4, 20, false);
    }

    public static Command scoreMidCubeLeft() {
        return new SetPositions(23, 0.1, -14, false);
    }

    public static Command scoreMidCubeRight() {
        return new SetPositions(23, 0.1, 14, false);
    }


    public static Command scoreCenterCone() {
        return new SetPositions(42, 1.5, 0, false);
    }

    public static Command scoreCenterCube() {
        return new SetPositions(23, 1.1, 0, false);
    }

    public static Command scoreMidConeCenter() {
        return new SetPositions(33, 0.75, 0, false);
    }

    public static Command scoreMidCubeCenter() {
        return new SetPositions(17, 0.4, 0, false);
    }


    //AUTOS
    public static Command scoreSubCone(){
        return new SetPositions(42, 1.15, 15);
    }

    public static Command scoreWallCone(){
        return new SetPositions(42, 1.15, 15, true);
    }

    public static Command scoreSubCube(){
        return new SetPositions(35, 0.7, -15, true);
    }

    public static Command scoreWallCube(){
        return new SetPositions(35, 1.0, 0, true); //changed for 2 piece mgeg
    }

    public static Command score180SubCone() {
        return new SetPositions(146.861282, 1.06, -195.2666, true);
    }

    public static Command score180WallCone() {
        return new SetPositions(146.861282, 1.06, 195.2666, true);
    }

    public static Command scoreSubCubeMid() {
        return new SetPositions(23, 0.4, 42, true);
    }

    public static Command scoreWallCubeMid() {
        return new SetPositions(23, 0.4, 42, true);
    }

    //ORIGINAL CONE AND CUBE SETPOINTS
    // public static Command coneSingleStation(){
    //     System.out.println("CONE SUBSTAION");
    //     return new ParallelCommandGroup(
    //         new SetPositions(130, 0, -90, true, false, true),
    //         new SetClaw(115),
    //         new RunIntake()
    //     );
    // }

    // public static Command cubeSingleStation(){
    //     System.out.println("CUBE SUBATATIONS");
    //     return new ParallelCommandGroup(
    //         new SetPositions(130, 0, 90, true, false, true),
    //         new SetClaw(80), 
    //         new RunIntake()
    //     );
    // }

    public static Command redConeSingleStation(){
        System.out.println("RED CONE SUBSTAION");
        return new ParallelCommandGroup(
            //new SetPositions(130, 0, -90, true, false, true),
            new SetPositions(130, 0, 90, false, false, true),
            new SetClaw(115),
            new RunIntake()
        );
    }

    public static Command redCubeSingleStation(){
        System.out.println("RED CUBE SUBATATIONS");
        return new ParallelCommandGroup(
            //new SetPositions(130, 0, 90, false, false, true),
            new SetPositions(17, 0, -90, false, false, true),
            new SetClaw(80), 
            new RunIntake()
        );
    }

    public static Command blueConeSingleStation(){
        System.out.println("BLUE CONE SUBSTAION");
        return new ParallelCommandGroup(
            //new SetPositions(130, 0, 90, false, false, true),
            new SetPositions(130, 0, -90, false, false, true),
            new SetClaw(115),
            new RunIntake()
        );
    }

    public static Command blueCubeSingleStation(){
        System.out.println("BLUE CONE SUBATATIONS");
        return new ParallelCommandGroup(
            //new SetPositions(17, 0, -90, false, false, true),
            new SetPositions(17, 0, 90, false, false, true),
            new SetClaw(80), 
            new RunIntake()
        );
    }

    public static Command cubeSingleTurret(){
        return new SetPositions(90, 0, 0, true, false, false);
    }

    public static Command coneDouble(){
        return new ParallelCommandGroup(
            new SetPositions(42, 0.8, 0, false, false, true),
            //was 43
            //67    new SetPositions(43, 0.8, 0, false, false, false),
            new SetClaw(15), 
            new RunIntake()
        );

    }

    /**
     * Opens the claw only for picking up cubes
     * @return
     */
    public static Command openClawCubeGround(){
        return new SetClaw(85, false);
    }

    public static Command groundCubeAuto(){
        return new ParallelCommandGroup(
            new SetPositions(-27.5, 0.65, 180),
            new SetClaw(80),
            new RunIntake()
        );
    }

    public static Command groundCubeAutoNoTurret(){
        return new ParallelCommandGroup(
            new SetTilt(-27),
            new SetExtend(0.65, false),
            new SetClaw(85),
            new RunIntake()
        );
    }

    public static Command groundCubeTele(){
        return new ParallelCommandGroup(
            new SetPositions(-27.5, 0.65, 0),
            new SetClaw(80),
            new RunIntake()
        );
    }

    public static Command groundConeTele(){
        return new ParallelCommandGroup(
            new SetPositions(-27, 0.65, 0),
            new SetClaw(85),
            new RunIntake()
        );
    }

    public static Command setShoot() {
        return new ParallelCommandGroup(
            new SetPositions(40, 1.0, 0),
            new SetClaw(69, true)
        );
    }

    public static Command setShootAuto() {
        return new ParallelCommandGroup(
            new SetPositions(30, 1.0, 0),
            new SetClaw(69, true)
        );
    }

}
