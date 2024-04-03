package frc.robot.Commands;
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Setpoints;
import frc.robot.Subsystems.Amp.AmpSubsystem;
import frc.robot.Subsystems.Feeder.FeederSubsystem;
import frc.robot.Subsystems.Intake.IntakeSubsystem;

public class IndexToAmp extends Command {

  private IntakeSubsystem intake;
  private AmpSubsystem amp;
  private FeederSubsystem feeder;

  /** Creates a new IntakeNote. */
  public IndexToAmp(IntakeSubsystem intake, AmpSubsystem amp, FeederSubsystem feeder){

    this.intake = intake;
    this.amp = amp;
    this.feeder = feeder;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake, amp, feeder);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (intake.isNotePresentTOF() || amp.isNotePresentTOF()) {
      //Leds.getInstance().hasGamePiece = noteDetected;
    }
        intake.setIntakeVoltage(Setpoints.intakingTargetVoltage);
        amp.setAmpVoltage(Setpoints.ampEjectTargetVoltage);
        feeder.setFeederVoltage(-Setpoints.indexingTargetVolts);
}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.stop();
    amp.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (amp.isNoteCenteredTOF()) {
      return true;
    }
    return false;
  }
}
