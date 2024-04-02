// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import com.ctre.phoenix6.SignalLogger;


import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Subsystems.Amp.AmpSubsystem;
import frc.robot.Subsystems.Feeder.FeederSubsystem;
import frc.robot.Subsystems.Intake.IntakeSubsystem;
import frc.robot.Subsystems.Pivot.PivotSubsystem;
import frc.robot.Subsystems.Shooter.ShooterSubsystem;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private IntakeSubsystem intake;
  private FeederSubsystem feeder;
  private AmpSubsystem amp;
  private PivotSubsystem pivot;
  private ShooterSubsystem shooter;

  private RobotContainer m_robotContainer;

  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();
    intake = new IntakeSubsystem();
    feeder = new FeederSubsystem();
    amp = new AmpSubsystem();
    pivot = new PivotSubsystem();
    shooter = new ShooterSubsystem();

    m_robotContainer.m_drivetrain.getDaqThread().setThreadPriority(99);

    SignalLogger.start();
  }
  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run(); 
    SmartDashboard.putBoolean("intake sensor", intake.isNotePresentTOF());
    SmartDashboard.putNumber("intake sensor range", intake.getRangeTOF());
    SmartDashboard.putBoolean("feeder sensor", feeder.isNotePresentTOF());
    SmartDashboard.putBoolean("feeder sensor centered", feeder.isNoteCenteredTOF());
    SmartDashboard.putNumber("feeder sensor range", feeder.getRangeTOF());
    SmartDashboard.putBoolean("amp sensor", amp.isNotePresentTOF());
    SmartDashboard.putBoolean("amp sensor centered", amp.isNotePresentTOF());
    SmartDashboard.putNumber("amp sensor range", amp.getRangeTOF());
    SmartDashboard.putNumber("shooter angle", pivot.getAngle().getDegrees());
    SmartDashboard.putNumber("shooter rpm", shooter.getVelocity()*60);
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    //m_robotContainer.drivetrain.setForwardHeading(Rotation2d.fromDegrees(DriverStation.getAlliance().orElse(Alliance.Red) == Alliance.Red ? 0 : 180));
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  @Override
  public void simulationPeriodic() {}
}
