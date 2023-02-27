// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.*;
import frc.robot.commands.ExtendArmManual;
import frc.robot.commands.FieldOrientedDrive;
import frc.robot.commands.ManualClaw;
import frc.robot.commands.FieldOrientedTurret;
import frc.robot.commands.TiltArmManual;
import frc.robot.commands.AutoEngage;
import frc.robot.commands.CharacterizeDrivetrainCommand;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.ManualTurret;
import frc.robot.commands.RobotOrientedDrive;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.ArmExtend;
import frc.robot.subsystems.ArmTilt;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Turret;
import frc.robot.Autos;

import frc.robot.commands.*;

import com.swervedrivespecialties.swervelib.DriveController;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final Autos auto = new Autos();
  private final ArmExtend extend = ArmExtend.getInstance();
  private final ArmTilt tilt = ArmTilt.getInstance();
  private final Turret turret = Turret.getInstance();
  private final DriveTrain driveTrain = DriveTrain.getInstance();
  private final Claw claw = Claw.getInstance();

  private final FieldOrientedDrive fieldDrive = new FieldOrientedDrive();
  private final RobotOrientedDrive robotDrive = new RobotOrientedDrive();
  private final CharacterizeDrivetrainCommand characterize = new CharacterizeDrivetrainCommand(driveTrain);

  private final TiltArmManual armTilt = new TiltArmManual();
  private final ExtendArmManual armExtension = new ExtendArmManual();
  private final ManualTurret manualTurret = new ManualTurret();
  private final ManualClaw manualClaw = new ManualClaw();

  private final FieldOrientedTurret fieldOrientedTurret = new FieldOrientedTurret();
  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController driverController =
      new CommandXboxController(XboxConstants.DRIVER_CONTROLLER_PORT);
  private final CommandXboxController operatorController =
      new CommandXboxController(XboxConstants.OPERATOR_CONTROLLER_PORT);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    configureDefaultCommands();
  }

  private void configureDefaultCommands() {
    // turret.setDefaultCommand(manualTurret);
    // tilt.setDefaultCommand(armTilt);
    // extend.setDefaultCommand(armExtension);
    driveTrain.setDefaultCommand(fieldDrive);
    // claw.setDefaultCommand(manualClaw);
  }


  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    new Trigger(m_exampleSubsystem::exampleCondition)
        .onTrue(new ExampleCommand(m_exampleSubsystem));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    // driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());

    // operatorController.leftTrigger().whileTrue(new CloseClaw());
    // operatorController.rightTrigger().whileTrue(new OpenClaw());

    driverController.leftStick().whileTrue(driveTrain.xOutCommand());
    driverController.y().whileTrue(new AutoEngage());
    operatorController.a().whileTrue(new SetArmPose(new Translation3d(-0.25, 0, 0.7), true));
    operatorController.b().whileTrue(new GroundPickup(false));
    operatorController.y().whileTrue(new SetArmPose(new Translation3d(1.1, 0, 1.2)));

    operatorController.back().whileTrue(new OpenClawCube());
    operatorController.start().whileTrue(new OpenClawConeSubstation());

    operatorController.pov(0).whileTrue(homing);
    operatorController.pov(90).whileTrue(new SetPositions(40, 1.05, 15));
    operatorController.pov(270).whileTrue(new SetPositions(40, 1.05, -15));

    operatorController.rightBumper().whileTrue(runIntake);
    operatorController.leftBumper().whileTrue(runIntakeReversed);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return auto.getAuto();
  }
}
