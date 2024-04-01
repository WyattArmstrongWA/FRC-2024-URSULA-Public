package frc.robot.Subsystems.Shooter;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Util.ErrorCheckUtil;
import frc.robot.Util.TalonFXFactory;
import frc.robot.Util.ErrorCheckUtil.CommonErrorNames;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase{

  private TalonFX shooterTalonLeader = configureShooterTalon(TalonFXFactory.createTalon(ShooterConstants.shooterTalonLeaderID,
    ShooterConstants.shooterTalonCANBus, ShooterConstants.kShooterConfiguration));
  private TalonFX shooterTalonFollower = configureShooterTalon(TalonFXFactory.createTalon(ShooterConstants.shooterTalonFollowerID,
      ShooterConstants.shooterTalonCANBus, ShooterConstants.kShooterConfiguration));

  public ShooterSubsystem() {
    shooterTalonFollower.setControl(ShooterConstants.followerControl);
  }

    /**
   * Set both shooter motors to the same speed
   * 
   * @param speed rotations per second
   */
  public void setVelocity(double speed) {

    shooterTalonLeader.setControl(ShooterConstants.shooterControl.withVelocity(speed));
    shooterTalonFollower.setControl(ShooterConstants.followerControl);
  }

  public void setShooterVoltage(double volts) {

    shooterTalonLeader.setControl(new VoltageOut(volts));
    shooterTalonFollower.setControl(ShooterConstants.followerControl);
  }

  public boolean isAtSetpoint() {
    return Math.abs(shooterTalonLeader.getClosedLoopError().getValue()) * 60 < ShooterConstants.shooterVelocityTolerance;
  }

  private TalonFX configureShooterTalon(TalonFX motor) {

    ErrorCheckUtil.checkError(
        motor.getVelocity().setUpdateFrequency(ShooterConstants.kShooterVelocityUpdateFrequency,
            Constants.kConfigTimeoutSeconds),
        CommonErrorNames.UpdateFrequency(motor.getDeviceID()));
    return motor;
  }

  public double getVelocity() {
    return shooterTalonFollower.getVelocity().getValue();
  }

  public void stop() {
    shooterTalonLeader.setControl(new DutyCycleOut(0));
  }

  @Override
  public void periodic() {
  }
}