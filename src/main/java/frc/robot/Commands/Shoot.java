// package frc.robot.Commands;
// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.Constants.Setpoints;
// import frc.robot.Subsystems.Amp.AmpSubsystem;
// import frc.robot.Subsystems.Feeder.FeederSubsystem;
// import frc.robot.Subsystems.Intake.IntakeSubsystem;
// import frc.robot.Subsystems.Pivot.PivotSubsystem;
// import frc.robot.Subsystems.Shooter.ShooterSubsystem;

// public class Shoot extends Command {

//   private FeederSubsystem feeder;
//   private ShooterSubsystem shooter;
//   private PivotSubsystem pivot;

//   /** Creates a new IntakeNote. */
//   public Shoot(PivotSubsystem pivot, ShooterSubsystem shooter, FeederSubsystem feeder){

//     this.pivot = pivot;
//     this.feeder = feeder;
//     this.shooter = shooter;
//     // Use addRequirements() here to declare subsystem dependencies.
//     addRequirements(pivot, shooter, feeder);
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {
//   }

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {
//     shooter.setVelocity()
//   }

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {
//     intake.stop();
//   }

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     if (intake.isNotePresentTOF()) {
//       return true;
//     }
//     return false;
//   }
// }

