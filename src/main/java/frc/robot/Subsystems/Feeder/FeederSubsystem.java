
package frc.robot.Subsystems.Feeder;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.playingwithfusion.TimeOfFlight;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FeederConstants;
import frc.robot.Util.TalonFXFactory;

public class FeederSubsystem extends SubsystemBase {

  private TalonFX feederTalon = configureFeederTalon(TalonFXFactory.createTalon(FeederConstants.feederTalonID,
      FeederConstants.feederTalonCANBus, FeederConstants.kFeederConfiguration));

  private TimeOfFlight feederSensor = new TimeOfFlight(FeederConstants.feederSensorID);

  public FeederSubsystem() {

    feederSensor.setRangingMode(FeederConstants.feederSensorRange, FeederConstants.feederSampleTime);
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

  public boolean isNotePresentTOF() {
    return feederSensor.getRange() < FeederConstants.isNotePresentTOF;
  }

  public double getRangeTOF() {
    return feederSensor.getRange();
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