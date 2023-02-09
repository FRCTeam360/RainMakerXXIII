// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.AutoDrive;
import frc.robot.subsystems.DriveTrain;

/** Add your docs here. */
public class DynamicPaths {

    DriveTrain driveTrain = DriveTrain.getInstance();

    public static Command dynPaths() {
        DriveTrain driveTrain = DriveTrain.getInstance();
        Pose2d pose2d = driveTrain.getPose();
        PathPlannerTrajectory trajectory = PathPlanner.generatePath(new PathConstraints(4, 3),
                new PathPoint(
                        new Translation2d(pose2d.getX(), pose2d.getY()), pose2d.getRotation(), new Rotation2d(0.0)),
                new PathPoint(new Translation2d(3.0, 3.0), Rotation2d.fromDegrees(45), Rotation2d.fromDegrees(0.0)));
        return new AutoDrive(trajectory, 0);
    }

}
