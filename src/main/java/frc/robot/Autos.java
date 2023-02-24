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

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
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

      put("Open Claw", new OpenClawCube());
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

    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  private Command getMyAuto() {
    List<PathPlannerTrajectory> epicPathGroup = PathPlanner.loadPathGroup("tsla stock is good",
        new PathConstraints(2, 3));
    // for(int i = 0; i<epicPathGroup.size(); i++){
    // PathPlannerTrajectory.transformTrajectoryForAlliance(epicPathGroup.get(i),
    // Alliance.Red);
    // }
    Command stockMarketCrash = autoBuilder.resetPose(epicPathGroup.get(0));
    Command pathTSLA1 = autoBuilder.followPath(epicPathGroup.get(0));
    Command pathTSLA2 = autoBuilder.followPath(epicPathGroup.get(1));

    return (new SetPositions(42, 1.1, -15)).andThen(new OpenClawCube()).andThen(new Homing())
        .andThen(stockMarketCrash)
        .andThen(new ParallelRaceGroup(pathTSLA1, new RunIntake(),
            new SequentialCommandGroup(/*new SetPositions(90, 0.1, -180), */ new WaitCommand(1),
                /* not working yet */ new SetArmPose(new Translation3d(0.3, 0, 0.05), false))))
        .andThen(new WaitCommand(.25)).andThen(pathTSLA2.alongWith(new Homing()).alongWith(new CloseClaw()));

    // return new SequentialCommandGroup(
    // new SetPositions(42, 1.1, -15), //dropping first game piece
    // new OpenClawCube(), //opening to cube position works for cones too
    // new Homing(),
    // stockMarketCrash,

    // new ParallelRaceGroup( //going to second game piece
    // pathTSLA1,
    // new SequentialCommandGroup(
    // new SetPositions(90, 0.1, -180),
    // new SetArmPose(new Translation3d(0.3, 0, 0.05), false)
    // )
    // ),

    // new ParallelRaceGroup( //getting second game piece
    // new WaitCommand(0.25),
    // eventMap.get("Close Claw")
    // ),

    // new ParallelRaceGroup( //going back
    // pathTSLA2,
    // new Homing()
    // ),

    // new SetPositions(42, 1.05, 15), //TODO: tune or swap w SetArmPose
    // eventMap.get("Open Claw"),
    // new Homing()
    // );
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
