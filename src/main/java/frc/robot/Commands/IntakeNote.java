package frc.robot.Commands;
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Setpoints;
import frc.robot.Subsystems.Amp.AmpSubsystem;
import frc.robot.Subsystems.Feeder.FeederSubsystem;
import frc.robot.Subsystems.Intake.IntakeSubsystem;

public class IntakeNote extends Command {

  private IntakeSubsystem intake;
  private FeederSubsystem feeder;
  private AmpSubsystem amp;

  /** Creates a new IntakeNote. */
  public IntakeNote(IntakeSubsystem intake, FeederSubsystem feeder, AmpSubsystem amp){

    this.intake = intake;
    this.feeder = feeder;
    this.amp = amp;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (intake.isNotePresentTOF() || feeder.isNotePresentTOF() || amp.isNotePresentTOF()) {
      //Leds.getInstance().hasGamePiece = noteDetected;
    }

    if (!intake.isNotePresentTOF()) {
      intake.setIntakeVoltage(Setpoints.intakingTargetVoltage);
    } else if (intake.isNotePresentTOF()) {
      intake.stop();
    }
    else if (feeder.isNotePresentTOF() || amp.isNotePresentTOF()) {
      intake.stop();
    } 
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (intake.isNotePresentTOF()) {
      return true;
    }
    return false;
  }
}
