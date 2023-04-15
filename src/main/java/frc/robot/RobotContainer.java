// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.*;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.utils.Setpoints;
import frc.robot.Autos;

import com.swervedrivespecialties.swervelib.DriveController;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
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
  private final Autos auto = new Autos();
  private final ArmExtend extend = ArmExtend.getInstance();
  private final ArmTilt tilt = ArmTilt.getInstance();
  private final Turret turret = Turret.getInstance();
  private final DriveTrain driveTrain = DriveTrain.getInstance();
  private final Claw claw = Claw.getInstance();
  private final Intake intake = Intake.getInstance();

  private final FieldOrientedDrive fieldDrive = new FieldOrientedDrive();
  // private final RobotOrientedDrive robotDrive = new RobotOrientedDrive();
  // private final CharacterizeDrivetrainCommand characterize = new CharacterizeDrivetrainCommand(driveTrain);

  private final RunIntake runIntake = new RunIntake();
  private final RunIntakeReversed runIntakeReversed = new RunIntakeReversed();

  private final TiltArmManual manualTilt = new TiltArmManual();
  // Controls inverted for ExtendArmManual, down is extend and up is retract
  private final ExtendArmManual manualExtend = new ExtendArmManual();
  private final ManualTurret manualTurret = new ManualTurret();
  private final ManualClaw manualClaw = new ManualClaw();

  private final SetPointArmExtension pidExtend = new SetPointArmExtension();
  private final SetPointArmTilt pidTilt = new SetPointArmTilt();
  private final SetPointClaw pidClaw = new SetPointClaw();
  private final SetPointTurret pidTurret = new SetPointTurret();

  private final TeleopArmPose teleopArmPose = new TeleopArmPose();
  private final FieldOrientedTurret fieldOrientedTurret = new FieldOrientedTurret();
  private final Homing homing = new Homing(false);

  private final Setpoints setpoints = new Setpoints();

  private final Shoot shoot = new Shoot();

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
    extend.setDefaultCommand(manualExtend); 
    tilt.setDefaultCommand(manualTilt); 
    driveTrain.setDefaultCommand(fieldDrive);
    claw.setDefaultCommand(manualClaw);
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

    driverController.leftStick().whileTrue(driveTrain.xOutCommand());
    driverController.y().whileTrue(new AutoEngage());

    // operatorController.a().and(() -> claw.isConeMode()).and(operatorController.start().negate()).whileTrue(Setpoints.coneSingleStation());
    // operatorController.a().and(() -> !claw.isConeMode()).and(operatorController.back().negate()).whileTrue(Setpoints.cubeSingleStation());

    operatorController.a().and(() -> DriverStation.getAlliance() == Alliance.Red).and(() -> claw.isConeMode()).and(operatorController.start().negate()).whileTrue(Setpoints.redConeSingleStation());
    operatorController.a().and(() -> DriverStation.getAlliance() == Alliance.Red).and(() -> !claw.isConeMode()).and(operatorController.back().negate()).whileTrue(Setpoints.redCubeSingleStation());

    operatorController.a().and(() -> DriverStation.getAlliance() == Alliance.Blue).and(() -> claw.isConeMode()).and(operatorController.start().negate()).whileTrue(Setpoints.blueConeSingleStation());
    operatorController.a().and(() -> DriverStation.getAlliance() == Alliance.Blue).and(() -> !claw.isConeMode()).and(operatorController.back().negate()).whileTrue(Setpoints.blueCubeSingleStation());
   

    operatorController.b().and(() -> claw.isConeMode()).whileTrue(Setpoints.groundConeTele());
    operatorController.b().and(() -> !claw.isConeMode()).whileTrue(Setpoints.groundCubeTele());

    operatorController.x().and(() -> claw.isConeMode()).whileTrue(Setpoints.coneDouble());
    
    //operatorController.y().whileTrue(new SetArmPose(new Translation3d(1.1, 0, 1.2)));
    operatorController.y().and(operatorController.leftBumper().negate()).whileTrue(Setpoints.setShoot());
    operatorController.y().and(operatorController.leftBumper()).whileTrue(shoot);

    // operatorController.back().and(operatorController.start().negate()).whileTrue(Setpoints.coneSingleTurret());
    // operatorController.start().and(operatorController.back().negate()).whileTrue(Setpoints.cubeSingleTurret());
    
    operatorController.pov(0).whileTrue(homing);

    // operatorController.pov(180).whileTrue(new InstantCommand(() -> turret.setPosition(0)));

    //HIGH SETPOINTS
    operatorController.pov(90).and(operatorController.leftStick().negate()).and(() -> claw.isConeMode()).whileTrue(Setpoints.scoreLeftCone());
    operatorController.pov(90).and(operatorController.leftStick().negate()).and(() -> !claw.isConeMode()).whileTrue(Setpoints.scoreLeftCube());

    operatorController.pov(270).and(operatorController.leftStick().negate()).and(() -> claw.isConeMode()).whileTrue(Setpoints.scoreRightCone()); //dpad left + NO back left paddle
    operatorController.pov(270).and(operatorController.leftStick().negate()).and(() -> !claw.isConeMode()).whileTrue(Setpoints.scoreRightCube()); //dpad left + NO back left paddle

    //MID SETPOINTS
    operatorController.pov(90).and(operatorController.leftStick()).and(() -> claw.isConeMode()).whileTrue(Setpoints.scoreMidConeRight());
    operatorController.pov(90).and(operatorController.leftStick()).and(() -> !claw.isConeMode()).whileTrue(Setpoints.scoreMidCubeRight());

    operatorController.pov(270).and(operatorController.leftStick()).and(() -> claw.isConeMode()).whileTrue(Setpoints.scoreMidConeLeft()); //dpad left + NO back left paddle
    operatorController.pov(270).and(operatorController.leftStick()).and(() -> !claw.isConeMode()).whileTrue(Setpoints.scoreMidCubeLeft()); //dpad left + NO back left paddle


    operatorController.rightBumper().whileTrue(runIntake);
    operatorController.leftBumper().and(operatorController.y().negate()).whileTrue(runIntakeReversed);  
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */

  public double calcFFTorqueMultiplier() {
    return Math.cos(Math.toRadians(this.tilt.getAngle())) * this.extend.getDistanceFromPivot();
  }
  public double extendFeedForward(){
    return Math.sin(Math.toRadians(this.tilt.getAngle()));
  }
  public Command getAutonomousCommand() {
    return auto.getAuto();
  }


}
