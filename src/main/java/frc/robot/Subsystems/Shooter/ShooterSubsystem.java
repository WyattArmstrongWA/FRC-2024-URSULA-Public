package frc.robot.Subsystems.Shooter;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
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

  private Orchestra m_orchestra = new Orchestra();

  public ShooterSubsystem() {
    shooterTalonFollower.setControl(ShooterConstants.followerControl);

    m_orchestra.addInstrument(shooterTalonLeader);
    m_orchestra.loadMusic("wonderwall.chrp");
  }

    /**
   * Set both shooter motors to the same speed
   * 
   * @param rpm rotations per minute
   */
  public void setVelocity(double rpm) {

   // shooterTalonLeader.setControl(ShooterConstants.shooterControl.withVelocity(-rps/2));
   shooterTalonLeader.setControl(ShooterConstants.shooterControl.withVelocity(rpm/60));
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

  public void singWonderwall() {
    m_orchestra.play();
  }

  public void stop() {
    shooterTalonLeader.setControl(new DutyCycleOut(0));
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("shooter rpm", getVelocity()*60);
  }

      public Command runShooterCommand(double velocity) {
        return new RunCommand(()->this.setVelocity(velocity), this);
    }

     public Command stopShooterCommand() {
        return new InstantCommand(()->this.stop(), this);
    }
}