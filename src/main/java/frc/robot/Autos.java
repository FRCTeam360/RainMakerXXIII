// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.commands.*;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import com.fasterxml.jackson.databind.ser.impl.FailingSerializer;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
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

      put("Open Claw", new OpenClawCubeGround());
      put("Close Claw", new CloseClaw());
      put("Set Point Arm Extension", new SetPointArmExtension());
      put("Set Point Arm Tilt", new SetPointArmTilt());
      put("Set Point Turret", new SetPointTurret());
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
    autoChooser.addOption("anywhere left", getAnywhereLeft());
    autoChooser.addOption("anywhere right", getAnywhereRight());
    autoChooser.addOption("engage from wall", getEngageFromWall());
    autoChooser.addOption("engage from loading", getEngageFromLoading());
    autoChooser.addOption("new 2 piece", getNew2Piece());

    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  private List<PathPlannerTrajectory> mirrorPathsForAlliance(List<PathPlannerTrajectory> mTrajectory) {
    ArrayList<PathPlannerTrajectory> newTrajectory = new ArrayList<PathPlannerTrajectory>();
    for (int i = 0; i < mTrajectory.size(); i++) {
      newTrajectory
          .add(PathPlannerTrajectory.transformTrajectoryForAlliance(mTrajectory.get(i), DriverStation.getAlliance()));
    }
    return newTrajectory;
  }

  private Command getAnywhereRight() {
    PathPlannerTrajectory path = PathPlanner.loadPath("anywhere", 2, 2);

    path = PathPlannerTrajectory.transformTrajectoryForAlliance(path, DriverStation.getAlliance());
    Command setPose = autoBuilder.resetPose(path);
    Command segment1 = autoBuilder.followPath(path);

    return setPose.andThen(new SetPositions(42, 1.1, 15, false)).andThen(new OpenClawCubeGround()).andThen(new Homing())
        .andThen(segment1);
  }

  private Command getAnywhereLeft() {
    PathPlannerTrajectory path = PathPlanner.loadPath("anywhere", 2, 2);

    path = PathPlannerTrajectory.transformTrajectoryForAlliance(path, DriverStation.getAlliance());
    Command setPose = autoBuilder.resetPose(path);
    Command segment1 = autoBuilder.followPath(path);

    return setPose.andThen(new SetPositions(42, 1.1, -15, false)).andThen(new OpenClawCubeGround())
        .andThen(new Homing()).andThen(segment1);
  }

  private Command getEngageFromWall() {
    PathPlannerTrajectory path = PathPlanner.loadPath("engage from wall", 2, 2);

    path = PathPlannerTrajectory.transformTrajectoryForAlliance(path, DriverStation.getAlliance());
    Command setPose = autoBuilder.resetPose(path);
    Command segment1 = autoBuilder.followPath(path);

    return setPose.andThen(new SetPositions(42, 1.1, 15, true)).andThen(new OpenClawCubeGround()).andThen(new Homing())
        .andThen(segment1).andThen(new AutoEngage());
  }

  private Command getEngageFromLoading() {
    PathPlannerTrajectory path = PathPlanner.loadPath("engage from loading", 2, 2);

    path = PathPlannerTrajectory.transformTrajectoryForAlliance(path, DriverStation.getAlliance());
    Command setPose = autoBuilder.resetPose(path);
    Command segment1 = autoBuilder.followPath(path);

    return setPose.andThen(new SetPositions(42, 1.1, -15, true)).andThen(new OpenClawCubeGround()).andThen(new Homing())
        .andThen(segment1).andThen(new AutoEngage());
  }

  private Command getMyMarioAuto() {

    List<PathPlannerTrajectory> luigi = PathPlanner.loadPathGroup("super mario bros", new PathConstraints(1, 3));
    luigi = mirrorPathsForAlliance(luigi);
    Command rainbowRoad = autoBuilder.resetPose(luigi.get(0));
    Command coconutMall = autoBuilder.followPath(luigi.get(0));
    // Command DKJungle = autoBuilder.followPath(luigi.get(1));
    // Command ShroomRidge = autoBuilder.followPath(luigi.get(2));
    return rainbowRoad.andThen(new InstantCommand(() -> driveTrain.setGyroOffset(180)))
        .andThen(new SetPositions(42, 1.1, 15, true).andThen(new OpenClawCubeGround()))
        .andThen((new Homing()).alongWith(coconutMall)).andThen(new AutoEngage());// .andThen(DKJungle).andThen(ShroomRidge).andThen(new
                                                                                  // AutoEngage());
  }

  private Command getMyAuto() {
    List<PathPlannerTrajectory> epicPathGroupInitial = PathPlanner.loadPathGroup("tsla stock is good",
        new PathConstraints(2, 3));
    // for(int i = 0; i<epicPathGroup.size(); i++){
    // PathPlannerTrajectory.transformTrajectoryForAlliance(epicPathGroup.get(i),
    // Alliance.Red);
    // }
    List<PathPlannerTrajectory> epicPathGroup = mirrorPathsForAlliance(epicPathGroupInitial);
    Command stockMarketCrash = autoBuilder.resetPose(epicPathGroup.get(0));
    Command pathTSLA1 = autoBuilder.followPath(epicPathGroup.get(0));
    Command pathTSLA2 = autoBuilder.followPath(epicPathGroup.get(1));

    // return (stockMarketCrash
    //     .andThen(new SetPositions(42, 1.05, -15, true)))
    //     .andThen(new OpenClawCubeGround())
    //     .andThen(new Homing())
    //     .andThen(
    //         new ParallelRaceGroup(
    //             pathTSLA1,
    //             new RunIntake(),
    //             new SequentialCommandGroup(/* new SetPositions(90, 0.1, -180), */
    //                 new SetPositions(0, 0.15, 180, true),
    //                 // new WaitCommand(1),
    //                 /* not working yet */
    //                 new SetPositions(-25, 0.6, 180, true)),
    //             new OpenClawCubeGround()))
    //     .andThen(new PrintCommand("pathTSLA1 done"))
    //     .andThen(
    //         new ParallelRaceGroup((new WaitCommand(.25)), (new RunIntake()), (new OpenClawCubeGround(false))))
    //     .andThen(
    //         pathTSLA2.alongWith(new Homing())// .raceWith(new CloseClaw())
    //     )
    //     .andThen(new PrintCommand("pathTSLA2 done"))
    //     .andThen(new SetPositions(42, 1.05, 15, true))
    //     .andThen(
    //         new ParallelRaceGroup(new OpenClawCubeGround(false), new RunIntakeReversed(), new WaitCommand(1)))
    //     .andThen(new Homing());

    return (stockMarketCrash
    .andThen(new SetPositions(42, 1.05, -15, true))
    .andThen(new OpenClawCubeGround(true))
    .andThen(new Homing())
    .andThen(new ParallelCommandGroup(
      pathTSLA1, 
      new SetPositions(0, 0.15, 180, true)
    ))
    );
  }

  private Command getNew2Piece() {
    List<PathPlannerTrajectory> epicPathGroupInitial = PathPlanner.loadPathGroup("new 2 piece",
        new PathConstraints(2, 3));
    // for(int i = 0; i<epicPathGroup.size(); i++){
    // PathPlannerTrajectory.transformTrajectoryForAlliance(epicPathGroup.get(i),
    // Alliance.Red);
    // }
    List<PathPlannerTrajectory> epicPathGroup = mirrorPathsForAlliance(epicPathGroupInitial);
    Command stockMarketCrash = autoBuilder.resetPose(epicPathGroup.get(0));
    Command part1 = autoBuilder.followPath(epicPathGroup.get(0));
    Command part2 = autoBuilder.followPath(epicPathGroup.get(1));
    Command part3 = autoBuilder.followPath(epicPathGroup.get(2));

    return (stockMarketCrash
    .andThen(new SetPositions(42, 1.05, -15, true))
    .andThen(new OpenClawCubeGround(true))
    .andThen(new Homing())
    .andThen(new ParallelCommandGroup(
      part1, 
      new SetPositions(0, 0.15, 180, true)
      .andThen(new SetPositions(-25, 0.6, 180))
    ))
    .andThen(new ParallelRaceGroup(
      part2, 
      new RunIntake(),
      new OpenClawCubeGround(false)
    ))
    .andThen(
      new ParallelCommandGroup(
        part3,
        new Homing()
      )
    )
    .andThen(
      new SetPositions(42, 1.05, 15, true)
    )
    .andThen(
      new ParallelRaceGroup(
        new RunIntakeReversed(),
        new WaitCommand(1)
      )
    )
    .andThen(new Homing())
    );
  }

  private static SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
      driveTrain::getPose,
      driveTrain::setPose,
      driveTrain.m_kinematics,
      new PIDConstants(5, 0, 0),
      new PIDConstants(1.5, 0, 0),
      driveTrain::setStates,
      eventMap,
      false,
      driveTrain);

  public Command getAuto() {
    return autoChooser.getSelected();
  }
}
