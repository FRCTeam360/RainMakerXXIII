// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.*;
import frc.robot.commands.ExtendArmManual;
import frc.robot.commands.FieldOrientedDrive;
import frc.robot.commands.TiltArmManual;
import frc.robot.commands.AutoArmPose;
import frc.robot.commands.CharacterizeDrivetrainCommand;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.ManualTurret;
import frc.robot.commands.PIDTuner;
import frc.robot.commands.RobotOrientedDrive;
import frc.robot.commands.SetPointArmTilt;
import frc.robot.commands.TeleopArmPose;
import frc.robot.commands.TestSetpoints;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.ArmExtend;
import frc.robot.subsystems.ArmTilt;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Turret;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final ArmExtend extend = ArmExtend.getInstance();
  private final ArmTilt tilt = ArmTilt.getInstance();
  private final Turret turret = Turret.getInstance();
  private final DriveTrain driveTrain = DriveTrain.getInstance();

  private final FieldOrientedDrive fieldDrive = new FieldOrientedDrive();
  private final RobotOrientedDrive robotDrive = new RobotOrientedDrive();
  private final CharacterizeDrivetrainCommand characterize = new CharacterizeDrivetrainCommand(driveTrain);

  private final TiltArmManual armTilt = new TiltArmManual();
  private final ExtendArmManual armExtension = new ExtendArmManual();
  private final ManualTurret manualTurret = new ManualTurret();
  private final TestSetpoints testSetpoints = new TestSetpoints();
  private final AutoArmPose autoArmPose = new AutoArmPose(0, 0);
  private final TeleopArmPose teleopArmPose = new TeleopArmPose();


  // getFFMultiplier.apply(tilt.getAngle(), extend.getDistanceFromPivot())
  private final PIDTuner pidTuner = new PIDTuner(extend, extend.getEncoder(), extend.getMotor(), extend.getPIDController(),
      this::extendFeedForward);

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController driverController =
      new CommandXboxController(XboxConstants.DRIVER_CONTROLLER_PORT);
  private final CommandXboxController operatorController =
      new CommandXboxController(XboxConstants.OPERATOR_CONTROLLER_PORT);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    configureDefaultCommands();
  }

  private void configureDefaultCommands() {
    turret.setDefaultCommand(manualTurret);
    extend.setDefaultCommand(armExtension); //armTilt
    tilt.setDefaultCommand(new SetPointArmTilt());//armExtension
    driveTrain.setDefaultCommand(fieldDrive);
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    new Trigger(m_exampleSubsystem::exampleCondition)
        .onTrue(new ExampleCommand(m_exampleSubsystem));
    driverController.a().whileTrue(new InstantCommand( () -> tilt.resetAngle()));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is
    // pressed,
    // cancelling on release.
    // driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());
    driverController.b().whileTrue(driveTrain.xOutCommand());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  // public Command getAutonomousCommand() {
  //   // An example command will be run in autonomous
  //   return Autos.exampleAuto(m_exampleSubsystem);
  // }

  public double calcFFTorqueMultiplier() {
    return Math.cos(Math.toRadians(this.tilt.getAngle())) * this.extend.getDistanceFromPivot();
  }
  public double extendFeedForward(){
    return Math.sin(Math.toRadians(this.tilt.getAngle()));
  }
}
