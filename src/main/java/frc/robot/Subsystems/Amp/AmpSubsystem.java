package frc.robot.Subsystems.Amp;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.playingwithfusion.TimeOfFlight;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AmpConstants;
import frc.robot.Util.TalonFXFactory;

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
   // m_orchestra.pause();
    ampTalon.setControl(new VoltageOut(volts, true, false, false, false));
  }

  public boolean isNotePresentTOF() {
    return ampSensor.getRange() < AmpConstants.isNotePresentTOF;
  }

  public double getRangeTOF() {
    return ampSensor.getRange();
  }

  public boolean isFeedToAmp() {
    return ampTalon.get() > 0.01;
  }

  public boolean isFeedToShooter() {
    return ampTalon.get() < -0.01;
  }

  public void stop() {
    ampTalon.setControl(AmpConstants.ampDutyCycle.withOutput(0));
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("amp speed", ampTalon.get());
    
  }

  private TalonFX configureAmpTalon(TalonFX motor) {
    return motor;
  }
}
    
