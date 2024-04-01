package frc.robot.subsystems.Amp;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.playingwithfusion.TimeOfFlight;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.TalonFXFactory;
import frc.robot.Constants.AmpConstants;
import frc.robot.Constants.Setpoints;

public class AmpSubsystem extends SubsystemBase {

  private TalonFX ampTalon = configureAmpTalon(TalonFXFactory.createTalon(AmpConstants.ampTalonID,
      AmpConstants.ampTalonCANBus, AmpConstants.kAmpConfiguration));

  private TimeOfFlight ampSensor = new TimeOfFlight(AmpConstants.ampSensorID);

  public AmpSubsystem() {

    ampSensor.setRangingMode(AmpConstants.ampSensorRange, AmpConstants.ampSampleTime);
    ampSensor.setRangeOfInterest(8, 8, 12, 12);
  }

  public void setAmpDutyCycle(double speed) {
    ampTalon.setControl(AmpConstants.ampDutyCycle.withOutput(speed));
  }

  public void setAmpTorqueControl(double amps) {
    ampTalon.setControl(AmpConstants.ampTorqueControl.withOutput(amps));
  }

  public void setAmpVoltage(double volts) {
    ampTalon.setControl(new VoltageOut(volts, true, false, false, false));
  }

  public boolean isNoteCenteredTOF() {
    return Math.abs(ampSensor.getRange() - AmpConstants.isNoteCenteredTOF) < AmpConstants.isNoteCenteredTOFTolerance;
  }

  public boolean isNotePresentTOF() {
    return ampSensor.getRange() < AmpConstants.isNotePresentTOF;
  }

  public void indexNoteToAmp() {

    if (ampSensor.getRange() > AmpConstants.isNotePresentTOF) {
      setAmpVoltage(Setpoints.indexingTargetVolts);
    } else if (isNoteCenteredTOF()) {
      stop();
    } else {
      if (ampSensor.getRange() > AmpConstants.isNoteCenteredTOF) {
        setAmpVoltage(Setpoints.indexingTargetVoltsSlow);
      } else if (ampSensor.getRange() < AmpConstants.isNoteCenteredTOF) {
        setAmpVoltage(-Setpoints.indexingTargetVoltsSlow);
      }
    }
  }

  public void stop() {
    ampTalon.setControl(AmpConstants.ampDutyCycle.withOutput(0));
  }

  @Override
  public void periodic() {
  }

  private TalonFX configureAmpTalon(TalonFX motor) {
    return motor;
  }
}
    
