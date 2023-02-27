package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

import java.util.function.DoubleSupplier;

import org.opencv.core.Mat;

public class FieldOrientedDrive extends CommandBase {
    private final DriveTrain driveTrain = DriveTrain.getInstance();

    private XboxController drivercont = new XboxController(0);

    public FieldOrientedDrive() {
        addRequirements(driveTrain);
    }

    private double getYWithDeadzone(){
        if(Math.abs(drivercont.getLeftX()) >= 0.125 || Math.abs(drivercont.getLeftY()) >= 0.125){
            return drivercont.getLeftY();
        } else {
            return 0.0;
        }
    }

    private double getXWithDeadzone(){
        if(Math.abs(drivercont.getLeftX()) >= 0.125 || Math.abs(drivercont.getLeftY()) >= 0.125){
            return drivercont.getLeftX();
        } else {
            return 0.0;
        }
    }

    @Override
    public void execute() {
        // You can use `new ChassisSpeeds(...)` for robot-oriented movement instead of field-oriented movement
        driveTrain.drive(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                    getYWithDeadzone() * getYWithDeadzone() * DriveTrain.MAX_VELOCITY_METERS_PER_SECOND * 0.8 * Math.signum(getYWithDeadzone()) * -1,
                    getXWithDeadzone() * getXWithDeadzone() * DriveTrain.MAX_VELOCITY_METERS_PER_SECOND * 0.8 * Math.signum(getXWithDeadzone()) * -1,
                    getWithDeadzone(drivercont.getRightX()) * getWithDeadzone(drivercont.getRightX()) * DriveTrain.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND * Math.signum(drivercont.getRightX()) * -1,
                        DriverStation.getAlliance() == Alliance.Red ? driveTrain.getGyroscopeRotation().minus(new Rotation2d(Math.PI)) : driveTrain.getGyroscopeRotation()
                )
        );

        if(drivercont.getPOV() == 0){
            driveTrain.zeroGyroscope();
        }
    }

    public double getWithDeadzone(double value){ //not sure if this is what you wanted me to do but i tried :(
        if(Math.abs(value) <= 0.125){
            return 0.0;
        } else {
            return value;
        }
    }

    @Override
    public void end(boolean interrupted) {
        driveTrain.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
    }
}