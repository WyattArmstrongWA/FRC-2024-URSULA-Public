package frc.robot.Commands;
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Setpoints;
import frc.robot.Subsystems.Amp.AmpSubsystem;
import frc.robot.Subsystems.Feeder.FeederSubsystem;
import frc.robot.Subsystems.Intake.IntakeSubsystem;

public class IndexToShooter extends Command {

  private IntakeSubsystem intake;
  private FeederSubsystem feeder;
  private AmpSubsystem amp;

  /** Creates a new IntakeNote. */
  public IndexToShooter(IntakeSubsystem intake, FeederSubsystem feeder, AmpSubsystem amp){

    this.intake = intake;
    this.feeder = feeder;
    this.amp = amp;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake, feeder, amp);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
         if(!feeder.isNotePresentTOF()) {
          intake.setIntakeVoltage(Setpoints.intakingTargetVoltage);
          feeder.setFeederVoltage(Setpoints.intakeFeedVolts);
          amp.setAmpVoltage(Setpoints.ampInjectTargetVoltage);
        } else {
          intake.stop();
          feeder.stop();
          amp.stop();
        }
}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.stop();
    feeder.stop();
    amp.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (feeder.isNotePresentTOF()) {
       return true;
    }
     return false;
   }
}