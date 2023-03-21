// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Claw.GamePiece;
import frc.robot.utils.Setpoints;
import frc.robot.commands.*;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import com.ctre.phoenix.platform.can.AutocacheState;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
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
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

public final class Autos {

  private static DriveTrain driveTrain = DriveTrain.getInstance();

  // this is the auto chooser
  private static SendableChooser<AutoMode> autoChooser;
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

      // put("Open Claw", new OpenClawCubeGround());
      put("Close Claw", new CloseClaw());
      put("Set Point Arm Extension", new SetPointArmExtension());
      put("Set Point Arm Tilt", new SetPointArmTilt());
      put("Set Point Turret", new SetPointTurret());
    }
  };

  public Autos() {
    autoChooser = new SendableChooser<>();
    // add auto options to chooser here
    autoChooser.addOption("coop mgeg", AutoMode.ONE_PIECE_MGEG);
    autoChooser.addOption("wall mgeg", AutoMode.ENGAGE_FROM_WALL);
    autoChooser.addOption("loading mgeg", AutoMode.ENGAGE_FROM_LOADING);

    autoChooser.addOption("wall 2 piece", AutoMode.NEW_2_PIECE_WALL);

    autoChooser.addOption("mobility left", AutoMode.ANYWHERE_LEFT);
    autoChooser.addOption("mobility right", AutoMode.ANYWHERE_RIGHT);

    autoChooser.addOption("loading 2 piece", AutoMode.NEW_2_PIECE);

    autoChooser.addOption("coop 0 piece mgeg", AutoMode.ENGAGE_ONLY);

    autoChooser.addOption("loading 2 piece mgeg", AutoMode.NEW_2_PIECE_BALANCE);

    autoChooser.addOption("null", AutoMode.NULL);

    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  private Command getExample() {
    return autoBuilder.fullAuto(pathGroup);
  }

  private Command getNotExample() {
    return autoBuilder.fullAuto(notPathGroup);
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

    // return setPose.andThen(new SetPositions(42, 1.1, 15, false)).andThen(new
    // OpenClawCubeGround()).andThen(new Homing())
    // .andThen(segment1);

    return setPose.andThen(Setpoints.scoreRightCone()).andThen(new RunIntake()).andThen(new Homing())
        .andThen(segment1);
  }

  private Command getAnywhereLeft() {
    PathPlannerTrajectory path = PathPlanner.loadPath("anywhere", 2, 2);

    path = PathPlannerTrajectory.transformTrajectoryForAlliance(path, DriverStation.getAlliance());
    Command setPose = autoBuilder.resetPose(path);
    Command segment1 = autoBuilder.followPath(path);

    // return setPose.andThen(new SetPositions(42, 1.1, -15, false)).andThen(new
    // OpenClawCubeGround())
    // .andThen(new Homing()).andThen(segment1);

    return setPose.andThen(Setpoints.scoreLeftCone()).andThen(new RunIntake().raceWith(new WaitCommand(.2))).andThen(new Homing())
        .andThen(segment1);
   
  }

  private Command getEngageFromWall() {
    PathPlannerTrajectory path = PathPlanner.loadPath("engage from wall", 2, 2);

    path = PathPlannerTrajectory.transformTrajectoryForAlliance(path, DriverStation.getAlliance());
    Command setPose = autoBuilder.resetPose(path);
    Command segment1 = autoBuilder.followPath(path);

    // return setPose.andThen(new SetPositions(42, 1.1, 15, true)).andThen(new
    // OpenClawCubeGround()).andThen(new Homing())
    // .andThen(segment1).andThen(new AutoEngage());

    return setPose.andThen(Setpoints.scoreWallCone()).andThen(new RunIntake()).andThen(new Homing())
        .andThen(segment1).andThen(new AutoEngage());
  }

  private Command getEngageFromLoading() {
    PathPlannerTrajectory path = PathPlanner.loadPath("engage from loading", 2, 2);

    path = PathPlannerTrajectory.transformTrajectoryForAlliance(path, DriverStation.getAlliance());
    Command setPose = autoBuilder.resetPose(path);
    Command segment1 = autoBuilder.followPath(path);

    // return setPose.andThen(new SetPositions(42, 1.1, -15, true)).andThen(new
    // OpenClawCubeGround()).andThen(new Homing())
    // .andThen(segment1).andThen(new AutoEngage());
    return setPose.andThen(Setpoints.scoreSubCone()).andThen(new RunIntake()).andThen(new Homing())
        .andThen(segment1).andThen(new AutoEngage());
  }

  private Command get1PieceMgeg() {
    List<PathPlannerTrajectory> luigi = PathPlanner.loadPathGroup("super mario bros", new PathConstraints(2, 3));
    luigi = mirrorPathsForAlliance(luigi);
    Command rainbowRoad = autoBuilder.resetPose(luigi.get(0));
    Command coconutMall = autoBuilder.followPath(luigi.get(0));
    // Command DKJungle = autoBuilder.followPath(luigi.get(1));
    // Command ShroomRidge = autoBuilder.followPath(luigi.get(2));
    // return rainbowRoad.andThen(new InstantCommand(() ->
    // driveTrain.setGyroOffset(180)))
    // .andThen(new SetPositions(42, 1.1, 15, true).andThen(new
    // OpenClawCubeGround()))
    // .andThen((new Homing()).alongWith(coconutMall)).andThen(new AutoEngage());//
    // .andThen(DKJungle).andThen(ShroomRidge).andThen(new
    // AutoEngage());

    return rainbowRoad.andThen(new InstantCommand(() -> driveTrain.setGyroOffset(180)))
        .andThen(Setpoints.scoreWallCone()).andThen(new RunIntake(GamePiece.CONE).raceWith(new WaitCommand(1.0)))
        .andThen((new Homing()).alongWith(coconutMall)).andThen(new AutoEngage());
  }

  private Command getEngageOnly() {

    List<PathPlannerTrajectory> luigi = PathPlanner.loadPathGroup("super mario bros", new PathConstraints(1, 3));
    luigi = mirrorPathsForAlliance(luigi);
    Command rainbowRoad = autoBuilder.resetPose(luigi.get(0));
    Command coconutMall = autoBuilder.followPath(luigi.get(0));
    // Command DKJungle = autoBuilder.followPath(luigi.get(1));
    // Command ShroomRidge = autoBuilder.followPath(luigi.get(2));
    return rainbowRoad.andThen(new InstantCommand(() -> driveTrain.setGyroOffset(180)))
        .andThen(coconutMall).andThen(new AutoEngage());// .andThen(DKJungle).andThen(ShroomRidge).andThen(new
                                                        // AutoEngage());
  }

  private Command getLoading2Piece() {
    List<PathPlannerTrajectory> epicPathGroup = DriverStation.getAlliance() == Alliance.Red
        ? mirrorPathsForAlliance(PathPlanner.loadPathGroup("new 2 piece",
            new PathConstraints(2, 3)))
        : PathPlanner.loadPathGroup("new 2 piece blue",
            new PathConstraints(2, 3));
    // for(int i = 0; i<epicPathGroup.size(); i++){
    // PathPlannerTrajectory.transformTrajectoryForAlliance(epicPathGroup.get(i),
    // Alliance.Red);
    // }
    Command stockMarketCrash = autoBuilder.resetPose(epicPathGroup.get(0));
    Command part1 = autoBuilder.followPath(epicPathGroup.get(0));
    Command part2 = autoBuilder.followPath(epicPathGroup.get(1));
    // Command part3 = autoBuilder.followPath(epicPathGroup.get(2));

    return (stockMarketCrash
        .andThen(new ParallelRaceGroup(
            Setpoints.scoreSubCone(),
            driveTrain.zeroModulesCommand()))
        // .andThen(new OpenClawCubeGround(true))
        .andThen(new ParallelRaceGroup(
            new RunIntake(),
            new WaitCommand(0.2)))
        .andThen(new Homing())
        .andThen(new ParallelCommandGroup(
            part1,
            new SetPositions(0, 0.15, 180, true)
                .andThen(Setpoints.groundCubeAuto())))

        // .andThen(new Homing())
        // .andThen(new ParallelCommandGroup(
        // new SetTurret(180, true, true),
        // part1,
        // // new SetPositions(0, 0.15, 180, true)
        // .andThen(Setpoints.groundCube())).raceWith(new
        // OpenClawCubeGround(false).raceWith(new RunIntake())))
        // // .andThen(new SetPositions(-27, 0.65, 180))))
        // // .andThen(new ParallelRaceGroup(
        // // part2,
        // // new RunIntake(),
        // // new OpenClawCubeGround(false)))
        .andThen(
            new ParallelCommandGroup(
                part2,
                new Homing()).raceWith(new RunIntake()))
        .andThen(
            new SetPositions(35, 0.8, 15, true))
        .andThen(
            new ParallelRaceGroup(
                new RunIntakeReversed(),
                new WaitCommand(1)))
        .andThen(new Homing()));
  }

  private Command getNew2PieceWall() {
    // List<PathPlannerTrajectory> epicPathGroup = DriverStation.getAlliance() ==
    // Alliance.Red
    // ? mirrorPathsForAlliance(PathPlanner.loadPathGroup("new 2 piece wall red",
    // new PathConstraints(2, 3)))
    // : PathPlanner.loadPathGroup("new 2 piece wall blue",
    // new PathConstraints(2, 3));
    List<PathPlannerTrajectory> epicPathGroup = mirrorPathsForAlliance(PathPlanner.loadPathGroup("new 2 piece wall red",
        new PathConstraints(2, 3)));
    // for(int i = 0; i<epicPathGroup.size(); i++){
    // PathPlannerTrajectory.transformTrajectoryForAlliance(epicPathGroup.get(i),
    // Alliance.Red);
    // }
    Command stockMarketCrash = autoBuilder.resetPose(epicPathGroup.get(0));
    Command part1 = autoBuilder.followPath(epicPathGroup.get(0));
    Command part2 = autoBuilder.followPath(epicPathGroup.get(1));
    // Command part3 = autoBuilder.followPath(epicPathGroup.get(2));

    return (stockMarketCrash
        .andThen(new ParallelRaceGroup(
            Setpoints.scoreWallCone(),
            driveTrain.zeroModulesCommand()))
        // .andThen(new OpenClawCubeGround(true))
        .andThen(new ParallelRaceGroup(
            new RunIntake(),
            new WaitCommand(0.2)))
        .andThen(new Homing())
        .andThen(new ParallelRaceGroup(
            part1,
            new SetPositions(0, 0.15, 180, true)
                .andThen(Setpoints.groundCubeAuto())))
        // .andThen(new SetPositions(-27, 0.65, 180))))
        // .andThen(new ParallelRaceGroup(
        // part2,
        // new RunIntake(),
        // new OpenClawCubeGround(false)))
        .andThen(
            new ParallelCommandGroup(
                part2,
                new Homing()).raceWith(new RunIntake()))
        .andThen(
            new SetPositions(35, 0.8, -15, true))
        .andThen(
            new ParallelRaceGroup(
                new RunIntakeReversed(),
                new WaitCommand(1)))
        .andThen(new Homing()));
  }

  private Command get2PieceAndBalance() {
    List<PathPlannerTrajectory> epicPathGroup = DriverStation.getAlliance() == Alliance.Red
        ? mirrorPathsForAlliance(PathPlanner.loadPathGroup("new 2 piece",
            new PathConstraints(2, 3)))
        : PathPlanner.loadPathGroup("new 2 piece blue",
            new PathConstraints(2, 3));
    // for(int i = 0; i<epicPathGroup.size(); i++){
    // PathPlannerTrajectory.transformTrajectoryForAlliance(epicPathGroup.get(i),
    // Alliance.Red);
    // }
    Command stockMarketCrash = autoBuilder.resetPose(epicPathGroup.get(0));
    Command part1 = autoBuilder.followPath(epicPathGroup.get(0));
    Command part2 = autoBuilder.followPath(epicPathGroup.get(1));
    // Command part3 = autoBuilder.followPath(epicPathGroup.get(2));

    return (stockMarketCrash
        .andThen(new ParallelRaceGroup(
            Setpoints.scoreSubCone(),
            driveTrain.zeroModulesCommand()))
        // .andThen(new OpenClawCubeGround(true))
        .andThen(new ParallelRaceGroup(
            new RunIntake(),
            new WaitCommand(0.2)))
        .andThen(
            // new ParallelRaceGroup(
            // (new SetTurret(-180, true, true)
            // .andThen(
            // Setpoints.groundCubeAuto())),
            // (new SetExtend(0.15, true)
            // .andThen(
            // part1
            // .alongWith(new SetTilt(0, true))
            // .alongWith(new SetExtend(0.15, true))))))
            new ParallelCommandGroup(
                // new SetExtend(0.15, true)
                // .andThen(
                new ParallelRaceGroup(
                    part1,
                    new ParallelCommandGroup(
                        new SetExtend(0.15, true),
                        new SetTurret(-180, true, true))
                        // new WaitUntilCommand(() ->
                        // Math.abs(Turret.getInstance().getAngleRelativeToRobot() + 180) <= 3))
                        .andThen(Setpoints.groundCubeAutoNoTurret())
                // )
                )// ,
                 // new SetTurret(-180, true, true)
            ))

        // .andThen(new Homing())
        // .andThen(new ParallelCommandGroup(
        // new SetTurret(180, true, true),
        // part1,
        // // new SetPositions(0, 0.15, 180, true)
        // .andThen(Setpoints.groundCube())).raceWith(new
        // OpenClawCubeGround(false).raceWith(new RunIntake())))
        // // .andThen(new SetPositions(-27, 0.65, 180))))
        // // .andThen(new ParallelRaceGroup(
        // // part2,
        // // new RunIntake(),
        // // new OpenClawCubeGround(false)))
        .andThen(
            new ParallelCommandGroup(
                part2,
                new Homing()).raceWith(new RunIntake()))
        .andThen(
            new SetPositions(35, 0.8, 15, true))
        .andThen(
            new ParallelRaceGroup(
                new RunIntakeReversed(),
                new WaitCommand(1)))
        .andThen(new Homing()));
  }

  private static SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
      driveTrain::getPose,
      driveTrain::setPose,
      driveTrain.m_kinematics,
      new PIDConstants(5, 0, 0),
      new PIDConstants(2.5, 0, 0),
      driveTrain::setStates,
      eventMap,
      false,
      driveTrain);

  public Command getAuto() {
    switch (autoChooser.getSelected()) {
      case CHOP_SUEY:
        return getExample();
      case LETS_GO_BRANDON:
        return getNotExample();
      // case TSLA_STOCK_IS_GOOD:
      //   return getMyAuto();
      case ONE_PIECE_MGEG:
        return get1PieceMgeg();
      case ANYWHERE_LEFT:
        return getAnywhereLeft();
      case ANYWHERE_RIGHT:
        return getAnywhereRight();
      case ENGAGE_FROM_WALL:
        return getEngageFromWall();
      case ENGAGE_FROM_LOADING:
        return getEngageFromLoading();
      case NEW_2_PIECE:
        return getLoading2Piece();
      case NEW_2_PIECE_WALL:
        return getNew2PieceWall();
      case ENGAGE_ONLY:
        return getEngageOnly();
      case NEW_2_PIECE_BALANCE:
        return get2PieceAndBalance();
      case NULL:
      default:
        return null;
    }
  }

  private enum AutoMode {
    CHOP_SUEY, LETS_GO_BRANDON, TSLA_STOCK_IS_GOOD, ONE_PIECE_MGEG, ANYWHERE_LEFT, ANYWHERE_RIGHT, ENGAGE_FROM_WALL,
    ENGAGE_FROM_LOADING, NEW_2_PIECE, NEW_2_PIECE_WALL, ENGAGE_ONLY, NEW_2_PIECE_BALANCE, NULL
  }
}
