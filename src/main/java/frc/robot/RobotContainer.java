// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.Supplier;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.AutoBuilder;
import static edu.wpi.first.wpilibj2.command.Commands.runOnce;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Commands.IndexToAmp;
import frc.robot.Commands.IndexToShooter;
import frc.robot.Commands.IntakeNote;
import frc.robot.Subsystems.Amp.AmpSubsystem;
import frc.robot.Subsystems.Drivetrain.CommandSwerveDrivetrain;
import frc.robot.Subsystems.Drivetrain.Telemetry;
import frc.robot.Subsystems.Feeder.FeederSubsystem;
import frc.robot.Subsystems.Intake.IntakeSubsystem;
import frc.robot.Subsystems.Pivot.PivotSubsystem;
import frc.robot.Subsystems.Shooter.ShooterSubsystem;
import frc.robot.Util.CommandXboxPS5Controller;
import frc.robot.Vision.Limelight;
import frc.robot.generated.TunerConstants;

public class RobotContainer {
  private IntakeSubsystem intake = new IntakeSubsystem();
  private FeederSubsystem feeder = new FeederSubsystem();
  private AmpSubsystem amp = new AmpSubsystem();
  private PivotSubsystem pivot = new PivotSubsystem();
  private ShooterSubsystem shooter = new ShooterSubsystem();
  private SendableChooser<Command> autoChooser;

  private double m_MaxSpeed = TunerConstants.kSpeedAt12VoltsMps; // Initial max is true top speed
  private final double MaxAngularRate = Math.PI * 1.5; // .75 rotation per second max angular velocity.  Adjust for max turning rate speed.
  private double m_AngularRate = MaxAngularRate; // This will be updated when turtle and reset to MaxAngularRate

  /* Setting up bindings for necessary control of the swerve drive platform */
  CommandXboxPS5Controller m_driverCtrl = new CommandXboxPS5Controller(0); // driver xbox controller
  CommandXboxPS5Controller m_operatorCtrl = new CommandXboxPS5Controller(1); // operator xbox controller
  CommandSwerveDrivetrain m_drivetrain = TunerConstants.DriveTrain; // drivetrain
  private Supplier<SwerveRequest> m_controlStyle;

  // Field-centric driving in Open Loop, can change to closed loop after characterization 
  // For closed loop replace DriveRequestType.OpenLoopVoltage with DriveRequestType.Velocity
  SwerveRequest.FieldCentric m_drive = new SwerveRequest.FieldCentric()
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
      .withDeadband(m_MaxSpeed * 0.1) // Deadband is handled on input
      .withRotationalDeadband(m_AngularRate * 0.1);

  Limelight vision = new Limelight(m_drivetrain);

  Telemetry logger = new Telemetry(m_MaxSpeed);

  Pose2d odomStart = new Pose2d(0, 0, new Rotation2d(0, 0));

  private void configureBindings() {
    double varAngle = 30;
    
    // reset the field-centric heading on start button press
    m_driverCtrl.start().onTrue(m_drivetrain.runOnce(() -> m_drivetrain.seedFieldRelative()));

    m_driverCtrl.a().whileTrue(new IntakeNote(intake, feeder, amp)).onFalse(runOnce(() -> intake.stop()));
    m_driverCtrl.rightTrigger().onTrue(runOnce(() -> feeder.setFeederVoltage(6))).onFalse(runOnce(() -> feeder.stop()));
    m_driverCtrl.rightBumper().whileTrue(new IndexToShooter(intake, feeder, amp)).onFalse(runOnce(() -> intake.stop()).alongWith(runOnce(() -> amp.stop())).alongWith(runOnce(() -> feeder.stop())));
    //m_driverCtrl.y().whileTrue(new Shoot(pivot, shooter, feeder, intake));
    // m_driverCtrl.y().whileTrue(new ScoreAmp(pivot, shooter, feeder, intake));
    //m_driverCtrl.rightBumper().whileTrue(new AlignToSpeaker());

    m_operatorCtrl.y().whileTrue(new IndexToAmp(intake, amp, feeder)).onFalse(runOnce(() -> intake.stop()).alongWith(runOnce(() -> amp.stop())).alongWith(runOnce(() -> feeder.stop())));
    m_operatorCtrl.povUp().whileTrue(new IndexToShooter(intake, feeder, amp)).onFalse(runOnce(() -> intake.stop()).alongWith(runOnce(() -> amp.stop())).alongWith(runOnce(() -> feeder.stop())));
    m_driverCtrl.x().onTrue(runOnce(() -> pivot.setAngle(Rotation2d.fromDegrees(varAngle))));
    m_driverCtrl.b().onTrue(runOnce(() -> pivot.setAngle(Rotation2d.fromDegrees(0))));
    //
    m_driverCtrl.rightTrigger().onTrue(runOnce(() -> feeder.setFeederVoltage(5))).onFalse(runOnce(() -> feeder.stop()));
    //m_driverCtrl.rightTrigger().onTrue(runOnce(() -> feeder.setFeederVoltage(5))).onFalse(runOnce(() -> feeder.stop()));
    m_driverCtrl.leftTrigger().whileTrue(runOnce(() -> shooter.setVelocity(-7200/60))).onFalse(runOnce(() -> shooter.stop()));
    m_driverCtrl.povUp().whileTrue(runOnce(() -> shooter.setVelocity(7200/60))).onFalse(runOnce(() -> shooter.stop()));

  }

  private void configureChooserBindings() {

        // Build an auto chooser. This will use Commands.none() as the default option.
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);
        // Set the initial Drive Control Style
        newControlStyle();
    }

  public RobotContainer() {
    // Detect if controllers are missing / Stop multiple warnings
    DriverStation.silenceJoystickConnectionWarning(true);

    // Build an auto chooser. This will use Commands.none() as the default option.
    autoChooser = AutoBuilder.buildAutoChooser();

    configureBindings();
    configureChooserBindings();

       if (Utils.isSimulation()) {
      m_drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(0)));
    }

    m_drivetrain.registerTelemetry(logger::telemeterize);
  }

  public Command getAutonomousCommand() {
    /* First put the drivetrain into auto run mode, then run the auto */
    return autoChooser.getSelected();
  }

  private void newControlStyle() {
    m_controlStyle = () -> m_drive.withVelocityX(-m_driverCtrl.getLeftY() * m_MaxSpeed * invertForAlliance()) // Drive forward -Y
    .withVelocityY(-m_driverCtrl.getLeftX() * m_MaxSpeed * invertForAlliance()) // Drive left with negative X (left)
    .withRotationalRate(-m_driverCtrl.getRightX() * m_AngularRate); // Drive counterclockwise with negative X (left)
// Specify the desired Control Style as the Drivetrain's default command
// Drivetrain will execute this command periodically
  m_drivetrain.setDefaultCommand(m_drivetrain.applyRequest(m_controlStyle).ignoringDisable(true));
  }

   // Inverts the joystick direction if on red alliance
    private double invertForAlliance() {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent() && alliance.get() == Alliance.Red) {
            return -1;
        }
        return 1;
    }
}
