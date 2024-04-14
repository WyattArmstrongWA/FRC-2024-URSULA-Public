package frc.robot.Subsystems.Elevator;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Util.ErrorCheckUtil;
import frc.robot.Util.TalonFXFactory;
import frc.robot.Util.ErrorCheckUtil.CommonErrorNames;


public class ElevatorSubsystem extends SubsystemBase {

      
  private TalonFX elevatorLeaderTalon = configureElevatorTalon(
      TalonFXFactory.createTalon(ElevatorConstants.elevatorLeaderTalonID,
          ElevatorConstants.elevatorTalonCANBus, ElevatorConstants.kElevatorConfiguration));

  private TalonFX elevatorFollowerTalon = configureElevatorTalon(
      TalonFXFactory.createTalon(ElevatorConstants.elevatorFollowerTalonID,
          ElevatorConstants.elevatorTalonCANBus, ElevatorConstants.kElevatorConfiguration));

  public ElevatorSubsystem() {
    elevatorFollowerTalon.setControl(ElevatorConstants.followerControl);
  }

  /**
   * Move Elevator to position
   * 
   * @param height in meters (0 to max height)
   */
  public void setHeight(double height) {

    elevatorLeaderTalon.setControl(
          ElevatorConstants.elevatorPositionControl.withPosition(ElevatorConstants.elevatorMetersToRotations(height)));
    elevatorFollowerTalon.setControl(ElevatorConstants.followerControl);
  }

  /**
   * Move elevator to home position (0)
   */
  public void stow() {
    setHeight(frc.robot.Constants.Setpoints.ElevatorStowHeight);
  }

  public void holdPosition() {

    elevatorLeaderTalon.setControl(new VoltageOut(ElevatorConstants.kElevatorConfiguration.Slot0.kG));
    elevatorFollowerTalon.setControl(ElevatorConstants.followerControl);
  }

  /**
   * Set all outputs to 0
   */
  public void stop() {

    elevatorLeaderTalon.setControl(new DutyCycleOut(0));
    elevatorFollowerTalon.setControl(new DutyCycleOut(0));
  }

  /**
   * Run elevator motors at a voltage
   * 
   * @param volts
   */
  public void setElevatorVoltage(double volts) {

    elevatorLeaderTalon.setControl(new VoltageOut(volts));
    elevatorFollowerTalon.setControl(ElevatorConstants.followerControl);
  }

  public void resetEncoderPosition(double height) {

    elevatorLeaderTalon.setPosition(ElevatorConstants.elevatorMetersToRotations(height));
    elevatorFollowerTalon.setPosition(ElevatorConstants.elevatorMetersToRotations(height));
  }

  public double getSetpointError() {
    return ElevatorConstants.elevatorRotationsToMeters(elevatorLeaderTalon.getClosedLoopError().getValue());
  }

  public double getLeaderPosition() {
    return ElevatorConstants.elevatorRotationsToMeters(elevatorLeaderTalon.getPosition().getValue());
  }

  public double getFollowerPosition() {
    return ElevatorConstants.elevatorRotationsToMeters(elevatorFollowerTalon.getPosition().getValue());
  }

  public boolean isAtSetpoint() {
    return Math.abs(getSetpointError()) < ElevatorConstants.heightErrorTolerance;
  }

   // To position for Intake, move Arm to INTAKE position
    public Command ampExtendCommand() {
        return new RunCommand(()-> this.setHeight(0.135), this)
            .until(()->this.isAtSetpoint());
    }  

    // To position for Intake, move Arm to INTAKE position
    public Command climbExtendCommand() {
      return new RunCommand(()-> this.setHeight(0.19), this)
          .until(()->this.isAtSetpoint());
  }  

  // To position for Intake, move Arm to INTAKE position
  public Command stowCommand() {
    return new RunCommand(()-> this.setHeight(0), this)
        .until(()->this.isAtSetpoint());
}  

  @Override
  public void periodic() {
    SmartDashboard.putNumber("ElevatorError", getSetpointError());
    SmartDashboard.putNumber("ElevatorFollowerCurrentHeight", getFollowerPosition());
    SmartDashboard.putNumber("ElevatorLeaderCurrentHeight", getLeaderPosition());
  }

    private TalonFX configureElevatorTalon(TalonFX motor) {

    ErrorCheckUtil.checkError(
        motor.getPosition().setUpdateFrequency(ElevatorConstants.kElevatorMidUpdateFrequency,
            Constants.kConfigTimeoutSeconds),
        CommonErrorNames.UpdateFrequency(motor.getDeviceID()));
    ErrorCheckUtil.checkError(
        motor.getClosedLoopError().setUpdateFrequency(ElevatorConstants.kElevatorMidUpdateFrequency,
            Constants.kConfigTimeoutSeconds),
        CommonErrorNames.UpdateFrequency(motor.getDeviceID()));
    ErrorCheckUtil.checkError(
        motor.getStatorCurrent().setUpdateFrequency(ElevatorConstants.kElevatorMidUpdateFrequency,
            Constants.kConfigTimeoutSeconds),
        CommonErrorNames.UpdateFrequency(motor.getDeviceID()));
    ErrorCheckUtil.checkError(
        motor.getReverseLimit().setUpdateFrequency(ElevatorConstants.kElevatorFastUpdateFrequency,
            Constants.kConfigTimeoutSeconds),
        CommonErrorNames.UpdateFrequency(motor.getDeviceID()));
    return motor;
  }
}
