// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands;

// import edu.wpi.first.wpilibj2.command.CommandBase;
// import frc.robot.subsystems.Limelight;

// //TODO I literally have 0 idea if this will work but idk its worth a shot
// public class TakeSnapshot extends CommandBase {
//   Limelight lime = Limelight.getInstance(); 
//   /** Creates a new TakeSnapshot. */
//   public TakeSnapshot() {
//     // Use addRequirements() here to declare subsystem dependencies.
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {}

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {
//     lime.snapshot(true);
//   }

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {
//     lime.snapshot(false);
//   }

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     return true;
//   }
// }
