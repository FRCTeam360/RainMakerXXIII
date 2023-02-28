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

      put("Open Claw", new OpenClawToHoldCube());
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
    autoChooser.addOption("tsla stock is good", getScore2AndBalanceOnAudienceSide());
    autoChooser.addOption("mario broskito", getMyMarioAuto());

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

  private Command getMyMarioAuto() {
    List<PathPlannerTrajectory> luigi = PathPlanner.loadPathGroup("super mario bros", new PathConstraints(3, 3));
    luigi = mirrorPathsForAlliance(luigi);
    Command rainbowRoad = autoBuilder.resetPose(luigi.get(0));
    Command coconutMall = autoBuilder.followPath(luigi.get(0));
    // Command DKJungle = autoBuilder.followPath(luigi.get(1));
    // Command ShroomRidge = autoBuilder.followPath(luigi.get(2));
    return rainbowRoad.andThen(new SetPositions(42, 1.1, 15, true).andThen(new OpenClawToHoldCube()))
        .andThen((new Homing()).alongWith(coconutMall)).andThen(new AutoEngage());// .andThen(DKJungle).andThen(ShroomRidge);//.andThen(new
                                                                                  // AutoEngage());
  }

  private Command getScore2AndBalanceOnAudienceSide() {
    List<PathPlannerTrajectory> rawPath = PathPlanner.loadPathGroup("Score 2 and Balance Audience Side",
        new PathConstraints(2, 3));

    List<PathPlannerTrajectory> mirroredPathGroup = mirrorPathsForAlliance(rawPath);
    Command resetBotPose = autoBuilder.resetPose(mirroredPathGroup.get(0))
        .andThen(() -> System.out.println("reset bot pose"));
    Command pathSegment1 = autoBuilder.followPath(mirroredPathGroup.get(0))
        .andThen(() -> System.out.println("finished path segment 1"));
    Command pathSegment2 = autoBuilder.followPath(mirroredPathGroup.get(1))
        .andThen(() -> System.out.println("finished path segment 2"));

    return (resetBotPose.andThen(new SetPositions(42, 1.05, -15, true))
        .andThen(new OpenClawToHoldCube())
        .andThen(
            pathSegment1
                .raceWith(new SequentialCommandGroup(
                    // pulls the arm into travel pose
                    new TravelPose(),
                    // rotates the turret into position to pickup cube
                    new TravelPose().alongWith(new SetTurretAngle(180, true)),
                    // lowers and extends arm to pick up cube
                    new SetPositions(-68, 0.6, 180, true)
                        // while running the intake and holding the claw at the correct position to pick
                        // up a cube
                        .alongWith(new RunIntake()).alongWith(new OpenClawToHoldCube(false)))))
        .andThen(new ParallelRaceGroup(
            new WaitCommand(.25),
            new RunIntake(),
            new OpenClawToHoldCube(false)))
        .andThen(
            pathSegment2
                .raceWith(new OpenClawToHoldCube(false))
                .alongWith(new TravelPose()
                    .andThen(new TravelPose().alongWith(new SetTurretAngle(15, true)))))
        .andThen(new SetPositions(42, 1.05, 15, true)
            .raceWith(new OpenClawToHoldCube(false)))
        .andThen(
            new ParallelRaceGroup(new OpenClawToHoldCube(false), new RunIntakeReversed(), new WaitCommand(1)))
        .andThen(new TravelPose()));
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
