// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.Supplier;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.ForwardReference;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.AutoBuilder;
//import com.pathplanner.lib.auto.NamedCommands;

import static edu.wpi.first.wpilibj2.command.Commands.runOnce;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
//import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Commands.IndexToAmp;
import frc.robot.Commands.IndexToShooter;
import frc.robot.Commands.IntakeNote;
import frc.robot.Subsystems.Amp.AmpSubsystem;
import frc.robot.Subsystems.Drivetrain.CommandSwerveDrivetrain;
import frc.robot.Subsystems.Drivetrain.Telemetry;
import frc.robot.Subsystems.Elevator.ElevatorSubsystem;
import frc.robot.Subsystems.Feeder.FeederSubsystem;
import frc.robot.Subsystems.Intake.IntakeSubsystem;
import frc.robot.Subsystems.Pivot.PivotSubsystem;
import frc.robot.Subsystems.Shooter.ShooterSubsystem;
import frc.robot.Util.CommandXboxPS5Controller;
import frc.robot.Vision.Limelight;
import frc.robot.generated.TunerConstants;

public class RobotContainer { 

  private SendableChooser<Command> autoChooser;

  // Track current MaxSpeed
  private double m_MaxSpeed = Constants.kMaxSpeed;
  // Track current AngularRate
  private double m_MaxAngularRate = Constants.kMaxAngularRate;

  /*
     * Driver/Operator controllers
     */
    CommandXboxPS5Controller m_driverCtrl = new CommandXboxPS5Controller(0);
    CommandXboxPS5Controller m_operatorCtrl = new CommandXboxPS5Controller(1);
    GenericHID m_driveRmbl = m_driverCtrl.getHID();
    GenericHID m_operatorRmbl = m_operatorCtrl.getHID();

// Drive Control style settings
private Supplier<SwerveRequest> m_controlStyle;

  /*
     * Swerve Drive Configuration
     */
    // Tuner Constants is a static class that defines the drivetrain constants
    // It is configured by the Phoenix Tuner X Swerve Project Generator
    CommandSwerveDrivetrain m_drivetrain = TunerConstants.DriveTrain;


  // Field-centric driving in Open Loop, can change to closed loop after characterization 
  // For closed loop replace DriveRequestType.OpenLoopVoltage with DriveRequestType.Velocity
  SwerveRequest.FieldCentric m_drive = new SwerveRequest.FieldCentric()
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
      .withDeadband(m_MaxSpeed * 0.2) // Deadband is handled on input
      .withRotationalDeadband(m_MaxAngularRate * 0.1);

  Telemetry m_logger = new Telemetry(m_MaxSpeed);

  Pose2d odomStart = new Pose2d(0, 0, new Rotation2d(0, 0));

  Limelight vision = new Limelight(m_drivetrain);

  IntakeSubsystem intake = new IntakeSubsystem();
  FeederSubsystem feeder = new FeederSubsystem();
  AmpSubsystem amp = new AmpSubsystem();
  PivotSubsystem pivot = new PivotSubsystem();
  ShooterSubsystem shooter = new ShooterSubsystem();
  ElevatorSubsystem elevator = new ElevatorSubsystem();


  public RobotContainer() {
    // Detect if controllers are missing / Stop multiple warnings
    DriverStation.silenceJoystickConnectionWarning(true);

     // Sets forward reference for drive to always be towards red alliance
     m_drive.ForwardReference = ForwardReference.RedAlliance;

     configureSmartDashboard();

     // Register NamedCommands for use in PathPlanner autos
     registerNamedCommands();

     // Configure Shuffleboard Chooser widgets
     configureChooserBindings();

     // Configure Driver and Operator controller buttons
     configureButtonBindings();

     // Set up the Telemetry function
     m_drivetrain.registerTelemetry(m_logger::telemeterize);

     // If running in Simulation, initialize the current pose
     if (Utils.isSimulation()) {
         m_drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(0)));
     }
  }

    private void registerNamedCommands() {
        // Register Named Commands for use in PathPlanner autos
    }

    private void configureChooserBindings() {

      // Build an auto chooser. This will use Commands.none() as the default option.
      autoChooser = AutoBuilder.buildAutoChooser();
      SmartDashboard.putData("Auto Chooser", autoChooser);
      // Set the initial Drive Control Style
      newControlStyle();
  }

   // Inverts the joystick direction if on red alliance
   private double invertForAlliance() {
    var alliance = DriverStation.getAlliance();
    if (alliance.isPresent() && alliance.get() == Alliance.Red) {
        return -1;
    }
    return 1;
}

// // Adds 180 degrees if on red alliance
// private double addForAlliance() {
//     var alliance = DriverStation.getAlliance();
//     if (alliance.isPresent() && alliance.get() == Alliance.Red) {
//         return 180;
//     }
//     return 0;
// }

private void configureSmartDashboard() {
    

    // if (RobotConstants.kIsAutoAimTuningMode) {
    //     SmartDashboard.putData("Shoot on Move Turning PID", m_head.HeadingController);
    //     SmartDashboard.putData("Static Turning PID", m_cardinal.HeadingController);
    // }
    // if (RobotConstants.kIsShooterTuningMode) {
    //     SmartDashboard.putData("Update Shooter Gains", m_shooterSubsystem.updateShooterGainsCommand());
    //     SmartDashboard.putData("Run Shooter", m_shooterSubsystem.runShooterCommand());
    //     SmartDashboard.putData("Stop Shooter", m_shooterSubsystem.stopShooterCommand());
    //     SmartDashboard.putData("Arm to Angle", m_armSubsystem.moveToDegreeCommand());
    // }
    // if (RobotConstants.kIsArmTuningMode) {
    //     SmartDashboard.putData("Move Arm To Setpoint", m_armSubsystem.tuneArmSetPointCommand());
    // }
    // if (RobotConstants.kIsTuningMode){
    //     //SmartDashboard.putNumber("Trap Speed", 22);
    // }
}

  private void configureButtonBindings() {

    double varAngle = 57.2;
    
    // reset the field-centric heading on start button press
    m_driverCtrl.start().onTrue(m_drivetrain.runOnce(() -> m_drivetrain.seedFieldRelative()));

    m_driverCtrl.a().whileTrue(new IntakeNote(intake)).onFalse(runOnce(() -> intake.stop()));
    m_driverCtrl.rightTrigger().onTrue(runOnce(() -> feeder.setFeederVoltage(6))).onFalse(runOnce(() -> feeder.stop()));
    m_driverCtrl.rightBumper().whileTrue(new IndexToShooter(intake, feeder, amp)).onFalse(runOnce(() -> intake.stop()).alongWith(runOnce(() -> amp.stop())).alongWith(runOnce(() -> feeder.stop())));
    m_driverCtrl.leftBumper().whileTrue(new IndexToAmp(intake, amp, elevator)).onFalse(runOnce(() -> intake.stop()).alongWith(runOnce(() -> amp.stop())).alongWith(runOnce(() -> feeder.stop())));
    m_driverCtrl.x().onTrue(runOnce(() -> pivot.setAngle(Rotation2d.fromDegrees(varAngle))));
    m_driverCtrl.b().onTrue(runOnce(() -> pivot.setAngle(Rotation2d.fromDegrees(1))));
    m_driverCtrl.povRight().onTrue(runOnce(() -> elevator.setHeight(.135)));
    m_driverCtrl.povLeft().onTrue(runOnce(() -> elevator.setHeight(0)));
    m_driverCtrl.rightTrigger().onTrue(runOnce(() -> feeder.setFeederVoltage(10))).onFalse(runOnce(() -> feeder.stop()));
    m_driverCtrl.leftTrigger().whileTrue(runOnce(() -> shooter.setVelocity(-5500/60))).onFalse(runOnce(() -> shooter.stop()));
    m_driverCtrl.povUp().whileTrue(runOnce(() -> shooter.setVelocity(-5600/60))).onFalse(runOnce(() -> shooter.stop()));
    m_driverCtrl.back().whileTrue(runOnce(() -> amp.setAmpVoltage(10))).onFalse(runOnce(() -> amp.stop()));
  }
    public Command getAutonomousCommand() {
        /* First put the drivetrain into auto run mode, then run the auto */
        return autoChooser.getSelected();
    }

    private void newControlStyle() {
        m_controlStyle = () -> m_drive.withVelocityX(-m_driverCtrl.getLeftY() * m_MaxSpeed * invertForAlliance()) // Drive forward -Y
                .withVelocityY(-m_driverCtrl.getLeftX() * m_MaxSpeed * invertForAlliance()) // Drive left with negative X (left)
                .withRotationalRate(-m_driverCtrl.getRightX() * m_MaxAngularRate); // Drive counterclockwise with negative X (left)
        // Specify the desired Control Style as the Drivetrain's default command
        // Drivetrain will execute this command periodically
        m_drivetrain.setDefaultCommand(m_drivetrain.applyRequest(m_controlStyle).ignoringDisable(true));
    }

    public Command rumbleDriverCommand() {
        return new RunCommand(() -> rumbleDriverCtrl()).withTimeout(2).finallyDo(() -> stopRumbleDriverCtrl());
    }

    public void rumbleDriverCtrl() {
        m_driveRmbl.setRumble(GenericHID.RumbleType.kLeftRumble, 1);
        m_operatorRmbl.setRumble(GenericHID.RumbleType.kLeftRumble, 1);
    }

    public void stopRumbleDriverCtrl() {
        m_driveRmbl.setRumble(GenericHID.RumbleType.kLeftRumble, 0);
        m_operatorRmbl.setRumble(GenericHID.RumbleType.kLeftRumble, 0);
    }

}
