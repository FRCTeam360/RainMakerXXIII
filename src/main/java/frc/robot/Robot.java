// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.utils.ArmPoseCalculator;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  private ArmPoseCalculator calculator = new ArmPoseCalculator();

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();


    
    // calculator.setRobotPose(new Pose3d(0.0, 0.0, 0.0, new Rotation3d(0, 0, 0)));
    // calculator.setTargetPose(new Pose3d(1.0, 1.0, 1.0, new Rotation3d(0, 0, 0)));

    calculator.setRobotTrans(new Translation3d(0,0,0));
    calculator.setTargetTrans(new Translation3d(1,-1,1));

    System.out.println("trans x: " + calculator.getX());
    System.out.println("trans y: " + calculator.getY());
    System.out.println("trans z: " + calculator.getZ());
    System.out.println("2d distance: " + calculator.get2dDistance());
    System.out.println("elevation: "+ calculator.getActualElevationAngleDegrees());
    System.out.println("turret rotation: " + calculator.getTurretRotation());
    System.out.println("extension: " + calculator.getExtendDistance());
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = Autos.getAuto();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
/*
 * blue alliance 3, Translation3d(X: 7.06, Y: 4.55, Z: 0.00)
blue alliance 2, Translation3d(X: 7.06, Y: 3.36, Z: 0.00)
blue alliance 1, Translation3d(X: 7.06, Y: 2.14, Z: 0.00)
blue alliance 0, Translation3d(X: 7.06, Y: 0.91, Z: 0.00)

red alliance 3, Translation3d(X: 8.82, Y: 4.55, Z: 0.00)
red alliance 2, Translation3d(X: 8.82, Y: 3.36, Z: 0.00)
red alliance 1, Translation3d(X: 8.82, Y: 2.14, Z: 0.00)
red alliance 0, Translation3d(X: 8.82, Y: 0.91, Z: 0.00)

GET BLUE NODES !!!
row:0, col: 0, Translation3d(X: 1.16, Y: 0.42, Z: 0.00)
row:0, col: 1, Translation3d(X: 1.16, Y: 1.07, Z: 0.00)
row:0, col: 2, Translation3d(X: 1.16, Y: 1.63, Z: 0.00)
row:0, col: 3, Translation3d(X: 1.16, Y: 2.19, Z: 0.00)
row:0, col: 4, Translation3d(X: 1.16, Y: 2.75, Z: 0.00)
row:0, col: 5, Translation3d(X: 1.16, Y: 3.31, Z: 0.00)
row:0, col: 6, Translation3d(X: 1.16, Y: 3.87, Z: 0.00)
row:0, col: 7, Translation3d(X: 1.16, Y: 4.42, Z: 0.00)
row:0, col: 8, Translation3d(X: 1.16, Y: 5.08, Z: 0.00)
row:1, col: 0, Translation3d(X: 0.80, Y: 0.51, Z: 0.87)
row:1, col: 1, Translation3d(X: 0.80, Y: 1.07, Z: 0.52)
row:1, col: 2, Translation3d(X: 0.80, Y: 1.63, Z: 0.87)
row:1, col: 3, Translation3d(X: 0.80, Y: 2.19, Z: 0.87)
row:1, col: 4, Translation3d(X: 0.80, Y: 2.75, Z: 0.52)
row:1, col: 5, Translation3d(X: 0.80, Y: 3.31, Z: 0.87)
row:1, col: 6, Translation3d(X: 0.80, Y: 3.87, Z: 0.87)
row:1, col: 7, Translation3d(X: 0.80, Y: 4.42, Z: 0.52)
row:1, col: 8, Translation3d(X: 0.80, Y: 4.98, Z: 0.87)
row:2, col: 0, Translation3d(X: 0.36, Y: 0.51, Z: 1.17)
row:2, col: 1, Translation3d(X: 0.36, Y: 1.07, Z: 0.83)
row:2, col: 2, Translation3d(X: 0.36, Y: 1.63, Z: 1.17)
row:2, col: 3, Translation3d(X: 0.36, Y: 2.19, Z: 1.17)
row:2, col: 4, Translation3d(X: 0.36, Y: 2.75, Z: 0.83)
row:2, col: 5, Translation3d(X: 0.36, Y: 3.31, Z: 1.17)
row:2, col: 6, Translation3d(X: 0.36, Y: 3.87, Z: 1.17)
row:2, col: 7, Translation3d(X: 0.36, Y: 4.42, Z: 0.83)
row:2, col: 8, Translation3d(X: 0.36, Y: 4.98, Z: 1.17)

GET RED NODES !!!
row:0, col: 0, Translation3d(X: 15.36, Y: 0.42, Z: 0.00)
row:0, col: 1, Translation3d(X: 15.36, Y: 1.07, Z: 0.00)
row:0, col: 2, Translation3d(X: 15.36, Y: 1.63, Z: 0.00)
row:0, col: 3, Translation3d(X: 15.36, Y: 2.19, Z: 0.00)
row:0, col: 4, Translation3d(X: 15.36, Y: 2.75, Z: 0.00)
row:0, col: 5, Translation3d(X: 15.36, Y: 3.31, Z: 0.00)
row:0, col: 6, Translation3d(X: 15.36, Y: 3.87, Z: 0.00)
row:0, col: 7, Translation3d(X: 15.36, Y: 4.42, Z: 0.00)
row:0, col: 8, Translation3d(X: 15.36, Y: 5.08, Z: 0.00)
row:1, col: 0, Translation3d(X: 15.74, Y: 0.51, Z: 0.87)
row:1, col: 1, Translation3d(X: 15.74, Y: 1.07, Z: 0.52)
row:1, col: 2, Translation3d(X: 15.74, Y: 1.63, Z: 0.87)
row:1, col: 3, Translation3d(X: 15.74, Y: 2.19, Z: 0.87)
row:1, col: 4, Translation3d(X: 15.74, Y: 2.75, Z: 0.52)
row:1, col: 5, Translation3d(X: 15.74, Y: 3.31, Z: 0.87)
row:1, col: 6, Translation3d(X: 15.74, Y: 3.87, Z: 0.87)
row:1, col: 7, Translation3d(X: 15.74, Y: 4.42, Z: 0.52)
row:1, col: 8, Translation3d(X: 15.74, Y: 4.98, Z: 0.87)
row:2, col: 0, Translation3d(X: 16.17, Y: 0.51, Z: 1.17)
row:2, col: 1, Translation3d(X: 16.17, Y: 1.07, Z: 0.83)
row:2, col: 2, Translation3d(X: 16.17, Y: 1.63, Z: 1.17)
row:2, col: 3, Translation3d(X: 16.17, Y: 2.19, Z: 1.17)
row:2, col: 4, Translation3d(X: 16.17, Y: 2.75, Z: 0.83)
row:2, col: 5, Translation3d(X: 16.17, Y: 3.31, Z: 1.17)
row:2, col: 6, Translation3d(X: 16.17, Y: 3.87, Z: 1.17)
row:2, col: 7, Translation3d(X: 16.17, Y: 4.42, Z: 0.83)
row:2, col: 8, Translation3d(X: 16.17, Y: 4.98, Z: 1.17)
 */