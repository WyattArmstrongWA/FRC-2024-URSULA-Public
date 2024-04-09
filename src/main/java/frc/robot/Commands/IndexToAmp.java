package frc.robot.Commands;
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Setpoints;
import frc.robot.Subsystems.Amp.AmpSubsystem;
import frc.robot.Subsystems.Elevator.ElevatorSubsystem;
import frc.robot.Subsystems.Intake.IntakeSubsystem;

public class IndexToAmp extends Command {

  private IntakeSubsystem intake;
  private AmpSubsystem amp;
  private ElevatorSubsystem elevator;

  /** Creates a new IntakeNote. */
  public IndexToAmp(IntakeSubsystem intake, AmpSubsystem amp, ElevatorSubsystem elevator){

    this.intake = intake;
    this.amp = amp;
    this.elevator = elevator;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake, amp, elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if(intake.isNotePresentTOF()) {
      elevator.setHeight(0);
      if(elevator.isAtSetpoint()) {
        intake.setIntakeVoltage(3);
        amp.setAmpVoltage(Setpoints.ampEjectTargetVoltage);
       } else {
         intake.stop();
         amp.setAmpVoltage(Setpoints.ampEjectTargetVoltage);
       }
     }
      else {
       elevator.setHeight(0.025);
       intake.setIntakeVoltage(Setpoints.intakingTargetVoltage);
       amp.setAmpVoltage(Setpoints.ampEjectTargetVoltage);
     }
}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    elevator.setHeight(0);
    amp.stop();
    intake.stop();
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
