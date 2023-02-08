package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;
import frc.SysIdGeneralMechanismLogger;

public class CharacterizeDrivetrainCommand extends CommandBase {
    private final DriveTrain drivetrain;

    private final SysIdGeneralMechanismLogger logger = new SysIdGeneralMechanismLogger();

    public CharacterizeDrivetrainCommand(DriveTrain drivetrain) {
        this.drivetrain = drivetrain;
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        // drivetrain.setPose(new Pose2d()); // Reset the drivetrain position
        logger.initLogging();
    }

    @Override
    public void execute() {
        System.out.println("char");
        // Update the data
        double position = drivetrain.getPose().getX();
        System.out.println("position: " + position);
        double velocity = drivetrain.getCurrentVelocity().vxMetersPerSecond;
        logger.log(position, velocity);

        // Get the voltage
        double voltage = logger.getMotorVoltage();

        // Drive at new voltage
        drivetrain.drive(
                new ChassisSpeeds(voltage / 12.0 * DriveTrain.MAX_VELOCITY_METERS_PER_SECOND, 0.0, 0.0));
    }

    @Override
    public void end(boolean interrupted) {
        logger.sendData();
        logger.reset();
        drivetrain.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
    }
}