// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.ExampleSubsystem;

import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public final class Autos {

  private static DriveTrain driveTrain = DriveTrain.getInstance();

  // this is the auto chooser
  private static SendableChooser<Command> autoChooser;
  // define autos here
  private static List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup("chop suey!",
      new PathConstraints(4, 3));
  private static List<PathPlannerTrajectory> notPathGroup = PathPlanner.loadPathGroup("lets go brandon",
      new PathConstraints(4, 3));

  // create events of commands here
  private static HashMap<String, Command> eventMap = new HashMap<>() {
    {
      put("tsla yay", new PrintCommand("tsla stock is good rn"));
      put("tsla halt", new WaitCommand(20));
    }
  };

  // instantiate autos here
  private final Command example = autoBuilder.fullAuto(pathGroup);
  private final Command notExample = autoBuilder.fullAuto(notPathGroup);

  public Autos() {
    autoChooser = new SendableChooser<>();
    // add auto options to chooser here
    autoChooser.addOption("chop suey!", example);
    autoChooser.addOption("lets go brandon", notExample);
    autoChooser.addOption("tsla stock is good", getMyAuto());

    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  private Command getMyAuto() {
    List<PathPlannerTrajectory> epicPathGroup = PathPlanner.loadPathGroup("tsla stock is good",
        new PathConstraints(4, 3));
    for(PathPlannerTrajectory epicPath : epicPathGroup){
      // epicPath = PathPlannerTrajectory.transformTrajectoryForAlliance(epicPath, Alliance.Red);
    }
    Command stockMarketCrash = autoBuilder.resetPose(PathPlannerTrajectory.transformTrajectoryForAlliance(epicPathGroup.get(0), Alliance.Red));
    Command pathTSLA1 = autoBuilder.followPath(PathPlannerTrajectory.transformTrajectoryForAlliance(epicPathGroup.get(0), Alliance.Red));
    Command pathTSLA2 = autoBuilder.followPath(PathPlannerTrajectory.transformTrajectoryForAlliance(epicPathGroup.get(1), Alliance.Red));

    return stockMarketCrash.andThen(pathTSLA1).andThen(pathTSLA2);
  }

  private static SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
      driveTrain::getPose,
      driveTrain::setPose,
      driveTrain.m_kinematics,
      new PIDConstants(3.1465, 0, 0),
      new PIDConstants(1.5, 0, 0),
      driveTrain::setStates,
      eventMap,
      driveTrain);

  public Command getAuto() {
    return autoChooser.getSelected();
  }
}
