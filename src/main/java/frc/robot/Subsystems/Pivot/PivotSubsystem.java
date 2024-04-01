package frc.robot.Subsystems.Pivot;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.PivotConstants;
import frc.robot.Constants.Setpoints;
import frc.robot.Util.ErrorCheckUtil;
import frc.robot.Util.TalonFXFactory;
import frc.robot.Util.ErrorCheckUtil.CommonErrorNames;

public class PivotSubsystem extends SubsystemBase {
  private final TalonFX pivotTalon = configurePivotTalon(TalonFXFactory.createTalon(PivotConstants.pivotTalonID, PivotConstants.pivotTalonCANBus, PivotConstants.kPivotConfiguration));

  private final CANcoder pivotEncoder = new CANcoder(PivotConstants.pivotEncoderID, PivotConstants.pivotEncoderCANBus);


  public PivotSubsystem() {

    pivotEncoder.getConfigurator().apply(PivotConstants.kPivotEncoderConfiguration, 1);
    this.pivotTalon.setPosition(this.pivotEncoder.getAbsolutePosition().getValue());
    this.pivotTalon.set(0);
  }

    /**
   * PID Pivot to position
   * 
   * @param rotations 0 to 1 rotations
   */
  private void setAngle(double position) {
    pivotTalon.setControl(PivotConstants.pivotPositionControl.withPosition(position));
  }

  /**
   * PID Pivot to position
   * 
   * @param rotations Rotation 2d
   */
  public void setAngle(Rotation2d position) {
    setAngle(position.getRotations());
  }

  public void stow() {
    setAngle(Setpoints.PivotStowAngle);
  }

  /**
   * Just PID to the current angle to hold position
   */
  public void holdPosition() {
    setAngle(getAngle());
  }

  public Rotation2d getSetpointError() {
    return Rotation2d.fromRotations(pivotTalon.getClosedLoopError().getValue());
  }

  public boolean isAtSetpoint() {
    return Math.abs(getSetpointError().getDegrees()) <= PivotConstants.angleErrorTolerance.getDegrees();
  }

  public Rotation2d getAngle() {
    return Rotation2d.fromRotations(pivotTalon.getPosition().getValue());
  }

  public double getVoltageOut() {
    return pivotTalon.getMotorVoltage().getValue();
  }

  public Rotation2d getAbsoluteEncoderPosition() {
    return Rotation2d.fromRotations(pivotEncoder.getAbsolutePosition().getValue()).minus(Rotation2d.fromRotations(PivotConstants.absoluteEncoderOffset.getRotations()));
  }

  public void stop() {
    pivotTalon.setControl(new DutyCycleOut(0));
  }

  public void setPivotVoltage(double volts) {
    pivotTalon.setControl(new VoltageOut(volts));
  }

  public void resetToAbsolute() {
    pivotTalon.setPosition(getAbsoluteEncoderPosition().getRotations());
  }

  @Override
  public void periodic() {

    SmartDashboard.putNumber("PivotError", getSetpointError().getDegrees());
    SmartDashboard.putNumber("PivotCurrentAngle", getAngle().getDegrees());
  }

    private TalonFX configurePivotTalon(TalonFX motor) {
    
        ErrorCheckUtil.checkError(
            motor.getPosition().setUpdateFrequency(PivotConstants.kPivotPositionUpdateFrequency,
                Constants.kConfigTimeoutSeconds),
            CommonErrorNames.UpdateFrequency(motor.getDeviceID()));
        ErrorCheckUtil.checkError(
            motor.getClosedLoopError().setUpdateFrequency(PivotConstants.kPivotErrorUpdateFrequency,
                Constants.kConfigTimeoutSeconds),
            CommonErrorNames.UpdateFrequency(motor.getDeviceID()));
        return motor;
      }
}