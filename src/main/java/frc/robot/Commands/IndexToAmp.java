package frc.robot.Commands;
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Setpoints;
import frc.robot.Subsystems.Amp.AmpSubsystem;
import frc.robot.Subsystems.Elevator.ElevatorSubsystem;
import frc.robot.Subsystems.Feeder.FeederSubsystem;
import frc.robot.Subsystems.Intake.IntakeSubsystem;

public class IndexToAmp extends Command {

  private IntakeSubsystem intake;
  private AmpSubsystem amp;
  private ElevatorSubsystem elevator;
  private FeederSubsystem feeder;
  private Boolean reindex;

  /** Creates a new IntakeNote. */
  public IndexToAmp(IntakeSubsystem intake, AmpSubsystem amp, ElevatorSubsystem elevator, FeederSubsystem feeder, Boolean reindex){

    this.intake = intake;
    this.amp = amp;
    this.elevator = elevator;
    this.feeder = feeder;
    this.reindex = reindex;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake, amp, elevator, feeder);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if(reindex==true) {
      intake.setIntakeVoltage(-Setpoints.intakingTargetVoltage);
    } else {
       intake.setIntakeVoltage(Setpoints.intakingTargetVoltage);
    }
      elevator.setHeight(0);
        amp.setAmpVoltage(Setpoints.ampEjectTargetVoltage);
        feeder.setFeederVoltage(-4);
  
       amp.setAmpVoltage(Setpoints.ampEjectTargetVoltage);
     }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {;
    amp.stop();
    intake.stop();
    feeder.stop();
  }

   // Returns true when the command should end.
   @Override
   public boolean isFinished() {
     if (amp.isNotePresentTOF()) {
       return true;
     }
   return false;
   }
}
