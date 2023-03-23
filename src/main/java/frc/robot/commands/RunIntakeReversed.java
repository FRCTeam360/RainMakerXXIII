// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Objects;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Claw.GamePiece;

public class RunIntakeReversed extends CommandBase {
  private final Intake intake = Intake.getInstance();
  private final Claw claw = Claw.getInstance();
  private Claw.GamePiece autoPieceType;

  /** Creates a new RunIntakeReversed. */
  public RunIntakeReversed() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake);
  }

  public RunIntakeReversed(Claw.GamePiece pieceType) {
    autoPieceType = pieceType;
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println(this.getClass().getSimpleName() + "started");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(!Objects.isNull(autoPieceType)) {
      if(autoPieceType == GamePiece.CONE) {
        intake.run(1.0);
      } else {
        intake.run(-1.0);
      }
    }

    if(Claw.getInstance().isConeMode()){
      intake.run(1.0);
    } else {
      intake.run(-1.0); // .5
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println(this.getClass().getSimpleName() + "finished");
    intake.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
