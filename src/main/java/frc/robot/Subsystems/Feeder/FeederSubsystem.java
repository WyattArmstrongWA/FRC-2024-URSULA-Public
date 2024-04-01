
package frc.robot.Subsystems.Feeder;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.playingwithfusion.TimeOfFlight;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FeederConstants;
import frc.robot.Constants.Setpoints;
import frc.robot.Util.TalonFXFactory;

public class FeederSubsystem extends SubsystemBase {

  private TalonFX feederTalon = configureFeederTalon(TalonFXFactory.createTalon(FeederConstants.feederTalonID,
      FeederConstants.feederTalonCANBus, FeederConstants.kFeederConfiguration));

  private TimeOfFlight feederSensor = new TimeOfFlight(FeederConstants.feederSensorID);

  public FeederSubsystem() {

    feederSensor.setRangingMode(FeederConstants.feederSensorRange, FeederConstants.feederSfeederleTime);
    feederSensor.setRangeOfInterest(8, 8, 12, 12);
  }

  public void setFeederDutyCycle(double speed) {
    feederTalon.setControl(FeederConstants.feederDutyCycle.withOutput(speed));
  }

  public void setFeederTorqueControl(double amps) {
    feederTalon.setControl(FeederConstants.feederTorqueControl.withOutput(amps));
  }

  public void setFeederVoltage(double volts) {
    feederTalon.setControl(new VoltageOut(volts, true, false, false, false));
  }

  public boolean isNoteCenteredTOF() {
    return Math.abs(feederSensor.getRange() - FeederConstants.isNoteCenteredTOF) < FeederConstants.isNoteCenteredTOFTolerance;
  }

  public boolean isNotePresentTOF() {
    return feederSensor.getRange() < FeederConstants.isNotePresentTOF;
  }

  public void indexNoteToFeeder() {

    if (feederSensor.getRange() > FeederConstants.isNotePresentTOF) {
      setFeederVoltage(Setpoints.indexingTargetVolts);
    } else if (isNoteCenteredTOF()) {
      stop();
    } else {
      if (feederSensor.getRange() > FeederConstants.isNoteCenteredTOF) {
        setFeederVoltage(Setpoints.indexingTargetVoltsSlow);
      } else if (feederSensor.getRange() < FeederConstants.isNoteCenteredTOF) {
        setFeederVoltage(-Setpoints.indexingTargetVoltsSlow);
      }
    }
  }

  public void stop() {
    feederTalon.setControl(FeederConstants.feederDutyCycle.withOutput(0));
  }

  @Override
  public void periodic() {
  }

  private TalonFX configureFeederTalon(TalonFX motor) {
    return motor;
  }

}