// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.commands.*;

import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.PrintCommand;

public final class Autos {

  private static DriveTrain driveTrain = DriveTrain.getInstance();

  //this is the auto chooser
  private static SendableChooser<Command> autoChooser = new SendableChooser<>();

  //define autos here
  private static List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup("example", new PathConstraints(4, 3));

  //create events of commands here
  private static HashMap<String, Command> eventMap = new HashMap<>() {{
    put("Open Claw", new OpenClaw());
    put("Close Claw", new CloseClaw());
    put("Set Point Arm Extension", new SetPointArmExtension());
    put("Set Point Arm Tilt", new SetPointArmTilt());
    put("Set Point Turret", new SetPointTurret());
  }};

  //instantiate autos here
  private final Command example = autoBuilder.fullAuto(pathGroup);

  private Autos() {
    //add auto options to chooser here
    autoChooser.addOption("example", example);
  }

  private static SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
    driveTrain::getPose, 
    driveTrain::setPose, 
    driveTrain.m_kinematics,
    new PIDConstants(0, 0, 0),
    new PIDConstants(0, 0, 0),
    driveTrain::setStates,
    eventMap,
    driveTrain
  );

  public static Command getAuto(){
    return autoChooser.getSelected();
  }
 }
