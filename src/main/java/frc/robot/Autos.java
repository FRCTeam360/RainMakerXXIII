// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.AutoEngage;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.ExampleSubsystem;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.wpilibj.DriverStation;
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
      put("engage", new AutoEngage());
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
    autoChooser.addOption("mario broskito", getMyMarioAuto());

    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  private List<PathPlannerTrajectory> mirrorPathsForAlliance(List<PathPlannerTrajectory> mTrajectory) {
    ArrayList<PathPlannerTrajectory> newTrajectory = new ArrayList<PathPlannerTrajectory>();
    for(int i = 0; i<mTrajectory.size(); i++) {
      newTrajectory.add(PathPlannerTrajectory.transformTrajectoryForAlliance(mTrajectory.get(i), DriverStation.getAlliance()));
    }
    return newTrajectory;
  }

  private Command getMyMarioAuto() {
    List<PathPlannerTrajectory> luigi = PathPlanner.loadPathGroup("super mario bros", new PathConstraints(4, 3));
    luigi = mirrorPathsForAlliance(luigi);
    Command rainbowRoad = autoBuilder.resetPose(luigi.get(0));
    Command coconutMall = autoBuilder.followPath(luigi.get(0));
    Command DKJungle = autoBuilder.followPath(luigi.get(1));
    Command ShroomRidge = autoBuilder.followPath(luigi.get(2));
    return rainbowRoad.andThen(coconutMall).andThen(DKJungle).andThen(ShroomRidge);
  }

  private Command getMyAuto() {
    List<PathPlannerTrajectory> epicPathGroup = PathPlanner.loadPathGroup("tsla stock is good",
        new PathConstraints(4, 3));
   // for(int i = 0; i<epicPathGroup.size(); i++){
     //PathPlannerTrajectory.transformTrajectoryForAlliance(epicPathGroup.get(i), Alliance.Red);
 //  }
    Command stockMarketCrash = autoBuilder.resetPose(epicPathGroup.get(0));
    Command pathTSLA1 = autoBuilder.followPath(epicPathGroup.get(0));
    Command pathTSLA2 = autoBuilder.followPath(epicPathGroup.get(1));

    return (new WaitCommand(1.0)).andThen(stockMarketCrash).andThen(pathTSLA1).andThen(new WaitCommand(.25)).andThen(pathTSLA2);
  }

  private static SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
      driveTrain::getPose,
      driveTrain::setPose,
      driveTrain.m_kinematics,
      new PIDConstants(3.1465, 0, 0),
      new PIDConstants(1.5, 0, 0),
      driveTrain::setStates,
      eventMap,
      true,
      driveTrain);

  public Command getAuto() {
    return autoChooser.getSelected();
  }
}
