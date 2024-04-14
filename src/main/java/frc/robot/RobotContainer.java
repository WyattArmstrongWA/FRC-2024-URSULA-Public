
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.ForwardReference;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Commands.IndexToAmp;
import frc.robot.Commands.IndexToShooter;
import frc.robot.Commands.IntakeNote;
import frc.robot.Commands.RefeedShooter;
import frc.robot.Commands.Shoot;
import frc.robot.Commands.SlowRefeed;
import frc.robot.Subsystems.Amp.AmpSubsystem;
import frc.robot.Subsystems.Drivetrain.CommandSwerveDrivetrain;
import frc.robot.Subsystems.Drivetrain.Telemetry;
import frc.robot.Subsystems.Elevator.ElevatorSubsystem;
import frc.robot.Subsystems.Feeder.FeederSubsystem;
import frc.robot.Subsystems.Intake.IntakeSubsystem;
import frc.robot.Subsystems.LED.LEDSubsystem;
import frc.robot.Subsystems.Pivot.PivotSubsystem;
import frc.robot.Subsystems.Shooter.ShooterSubsystem;
import frc.robot.Util.AprilTagLock;
import frc.robot.Util.CommandXboxPS5Controller;
import frc.robot.Util.TunableNumber;
import frc.robot.Vision.Limelight;
import frc.robot.generated.TunerConstants;

import static edu.wpi.first.wpilibj2.command.Commands.runOnce;


public class RobotContainer { 

  private SendableChooser<Command> autoChooser;

    private boolean speakerMode = true;
  private boolean ampMode = false;
  private boolean manualMode = false;

  private BooleanSupplier speakerModeSupplier = () -> speakerMode;
  private BooleanSupplier ampModeSupplier = () -> ampMode;

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
      .withDeadband(m_MaxSpeed * 0.08) // Deadband is handled on input
      .withRotationalDeadband(m_MaxAngularRate * 0.09);

    SwerveRequest.FieldCentricFacingAngle m_head = new SwerveRequest.FieldCentricFacingAngle()
      .withDriveRequestType(DriveRequestType.Velocity);
      SwerveRequest.FieldCentricFacingAngle m_cardinal = new SwerveRequest.FieldCentricFacingAngle()
      .withDriveRequestType(DriveRequestType.Velocity);

  Telemetry m_logger = new Telemetry(m_MaxSpeed);

   Limelight vision = new Limelight(m_drivetrain);

  Pose2d odomStart = new Pose2d(0, 0, new Rotation2d(0, 0));

  TunableNumber angle = new TunableNumber("Smart angle", 52);
    TunableNumber rpm = new TunableNumber("Smart rpm", 2200);

  //VisionSubsystem vision = new VisionSubsystem(m_drivetrain);

  IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();
  AmpSubsystem m_ampSubsystem = new AmpSubsystem();
  PivotSubsystem m_pivotSubsystem = new PivotSubsystem();
  ShooterSubsystem m_shooterSubsystem = new ShooterSubsystem();
  ElevatorSubsystem m_elevatorSubsystem = new ElevatorSubsystem();
  FeederSubsystem m_feederSubsystem = new FeederSubsystem();
  LEDSubsystem m_ledSubsystemSubsystem = new LEDSubsystem(m_intakeSubsystem, m_shooterSubsystem, m_drivetrain, m_ampSubsystem, m_feederSubsystem);


  public RobotContainer() {
    // Detect if controllers are missing / Stop multiple warnings
    DriverStation.silenceJoystickConnectionWarning(true);

     // Sets forward reference for drive to always be towards red alliance
     m_drive.ForwardReference = ForwardReference.RedAlliance;

     /* Dynamic turning PID */
     m_head.ForwardReference = ForwardReference.RedAlliance;
     m_head.HeadingController.setP(20);
     m_head.HeadingController.setI(0);
     m_head.HeadingController.setD(0); 
     m_head.HeadingController.enableContinuousInput(-Math.PI, Math.PI);
     m_head.HeadingController.setTolerance(Units.degreesToRadians(0.5));

     /* Static turning PID */
     m_cardinal.ForwardReference = ForwardReference.RedAlliance;
     m_cardinal.HeadingController.setP(0.5);
     m_cardinal.HeadingController.setI(0);
     m_cardinal.HeadingController.setD(0);
     m_cardinal.HeadingController.enableContinuousInput(-Math.PI, Math.PI);
     m_cardinal.HeadingController.setTolerance(Units.degreesToRadians(0.01));

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
        NamedCommands.registerCommand("ShootWait", new Shoot(m_shooterSubsystem, m_pivotSubsystem, vision, m_feederSubsystem, true).withTimeout(3));
        NamedCommands.registerCommand("Intake", new IndexToShooter(m_intakeSubsystem, m_feederSubsystem, m_ampSubsystem, true).andThen(new RefeedShooter(m_intakeSubsystem, m_feederSubsystem, m_ampSubsystem)).andThen(new SlowRefeed(m_intakeSubsystem, m_feederSubsystem, m_ampSubsystem)).withTimeout(3));

        
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

public boolean isAmpMode() {
  return ampMode;
}

public boolean rpmSetpoint() {
  return m_shooterSubsystem.isAtSetpoint();
}

public boolean angleSetpoint() {
  return m_pivotSubsystem.isAtSetpoint();
}

  private BooleanSupplier angleSet = () -> angleSetpoint();
  private BooleanSupplier rpmSet = () -> rpmSetpoint();

  // Adds 180 degrees if on red alliance
private double addForAlliance() {
     var alliance = DriverStation.getAlliance();
     if (alliance.isPresent() && alliance.get() == Alliance.Red) {
         return 180;
     }
     return 0;
 }

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

    m_operatorCtrl.y().onTrue(runOnce(() -> {
      speakerMode = true;
      ampMode = false;
    }));


    m_operatorCtrl.y().whileTrue(new IndexToShooter(m_intakeSubsystem, m_feederSubsystem, m_ampSubsystem, true).andThen(new RefeedShooter(m_intakeSubsystem, m_feederSubsystem, m_ampSubsystem)).andThen(new SlowRefeed(m_intakeSubsystem, m_feederSubsystem, m_ampSubsystem))).onFalse(runOnce(() -> m_intakeSubsystem.stop()).alongWith(runOnce(() -> m_ampSubsystem.stop())).alongWith(runOnce(() -> m_feederSubsystem.stop())));
    m_operatorCtrl.a().whileTrue(new IndexToAmp(m_intakeSubsystem, m_ampSubsystem, m_elevatorSubsystem, m_feederSubsystem, true)).onFalse(runOnce(() -> m_intakeSubsystem.stop()).alongWith(runOnce(() -> m_ampSubsystem.stop())).alongWith(runOnce(() -> m_feederSubsystem.stop())));

    m_operatorCtrl.a().onTrue(runOnce(() -> {
      speakerMode = false;
      ampMode = true;
    }));

    
    
    // reset the field-centric heading on start button press
    m_driverCtrl.start().onTrue(m_drivetrain.runOnce(() -> m_drivetrain.seedFieldRelative()));

    m_driverCtrl.rightBumper().and(ampModeSupplier).whileTrue(m_drivetrain.applyRequest(
      () -> m_cardinal.withVelocityX(-m_driverCtrl.getLeftY() * Constants.kMaxSpeed * invertForAlliance())
              .withVelocityY(-m_driverCtrl.getLeftX() * Constants.kMaxSpeed * invertForAlliance())
              .withTargetDirection(Rotation2d.fromDegrees(-90 + addForAlliance()))
              .withDeadband(Constants.kMaxSpeed * 0.1)
              .withRotationalDeadband(m_MaxAngularRate * 0.1)));

    // // Driver: When LeftTrigger is pressed, lower the Arm and then run the Intake
    //     // and Stage until a Note is found and then Rumble the driver controller for 1/2
    //     // sec
    //     m_driverCtrl.leftTrigger(0.05)
    //     .whileTrue(m_pivotSubsystem.prepareForIntakeCommand()
    //             .andThen(new IntakeNote(m_intakeSubsystem))
    //             .andThen(rumbleDriverCommand()));
    
    // // Driver: When RightTrigger is pressed, release Note to m_shooterSubsystem, then lower Arm
    // m_driverCtrl.rightTrigger(0.05)
    // .onTrue(m_feederSubsystem.feedWithBeam()
    //         .withTimeout(2)
    //         .andThen(m_pivotSubsystem.prepareForIntakeCommand()));

            // m_driverCtrl.rightBumper().whileTrue(Commands.parallel(
            //     new Shoot(m_drivetrain, m_feederSubsystem, m_pivotSubsystem, m_shooterSubsystem, true)
            //             .andThen(m_pivotSubsystem.prepareForIntakeCommand()),
            //     m_drivetrain.applyRequest(
            //             () -> m_head.withVelocityX(-m_driverCtrl.getLeftY() * m_MaxSpeed * .75 * invertForAlliance())
            //                     .withVelocityY(-m_driverCtrl.getLeftX() * m_MaxSpeed * .75 * invertForAlliance())
            //                     .withTargetDirection(m_drivetrain.getVelocityOffset())
            //                     .withDeadband(Constants.kMaxSpeed * 0.1))));

            //                     m_driverCtrl.rightBumper().onFalse(m_shooterSubsystem.stopShooterCommand());

    // // Operator: When A button is pressed, run Shooter
    // m_operatorCtrl.x().whileTrue(m_shooterSubsystem.runShooterCommand(5000)
    // .withInterruptBehavior(Command.InterruptionBehavior.kCancelSelf));

    // // Stop the m_shooterSubsystem
    // m_operatorCtrl.b().onTrue(m_shooterSubsystem.stopShooterCommand());

    // // Operator: Right D-Pad Button: Elevator to Climb Position (when pressed)
    // m_operatorCtrl.povRight().onTrue(m_elevatorSubsystem.climbExtendCommand());

    // // Operator: Up D-Pad Button: Elevator to Amp Position (when pressed)
    // m_operatorCtrl.povUp().onTrue(m_elevatorSubsystem.ampExtendCommand());

    // // Operator: down D-Pad Button: Elevator to Stowed Position (when pressed)
    // m_operatorCtrl.povDown().onTrue(m_elevatorSubsystem.stowCommand());

   // m_driverCtrl.rightBumper().whileTrue(new ShootSequence(m_shooterSubsystem, m_pivotSubsystem, vision, m_feederSubsystem)).onFalse(runOnce(() -> m_shooterSubsystem.stop()));

    // Driver: While Left Bumper is held, reduce speed by 50%
    m_driverCtrl.leftBumper().onTrue(runOnce(() -> m_MaxSpeed = Constants.kMaxSpeed * .25)
    .andThen(() -> m_MaxAngularRate = Constants.kMaxAngularRate * .25));
m_driverCtrl.leftBumper().onFalse(runOnce(() -> m_MaxSpeed = Constants.kMaxSpeed)
    .andThen(() -> m_MaxAngularRate = Constants.kMaxAngularRate));
    
    
   // m_driverCtrl.a().whileTrue(new IntakeNote(m_intakeSubsystem));
    m_driverCtrl.y().and(speakerModeSupplier).whileTrue(new Shoot(m_shooterSubsystem, m_pivotSubsystem, vision, m_feederSubsystem, false)).onFalse(runOnce(() -> m_shooterSubsystem.stop()));
     m_driverCtrl.povDown().onTrue(runOnce(() -> m_feederSubsystem.setFeederVoltage(10))).onFalse(runOnce(() -> m_feederSubsystem.stop()));

    m_driverCtrl.a().and(speakerModeSupplier).whileTrue(new IndexToShooter(m_intakeSubsystem, m_feederSubsystem, m_ampSubsystem, false).andThen(new RefeedShooter(m_intakeSubsystem, m_feederSubsystem, m_ampSubsystem)).andThen(new SlowRefeed(m_intakeSubsystem, m_feederSubsystem, m_ampSubsystem))).onFalse(runOnce(() -> m_intakeSubsystem.stop()).alongWith(runOnce(() -> m_ampSubsystem.stop())).alongWith(runOnce(() -> m_feederSubsystem.stop())));
     m_driverCtrl.a().and(ampModeSupplier).whileTrue(new IndexToAmp(m_intakeSubsystem, m_ampSubsystem, m_elevatorSubsystem, m_feederSubsystem, false)).onFalse(runOnce(() -> m_intakeSubsystem.stop()).alongWith(runOnce(() -> m_ampSubsystem.stop())).alongWith(runOnce(() -> m_feederSubsystem.stop())));
     m_driverCtrl.x().onTrue(runOnce(() -> m_pivotSubsystem.setAngle(Rotation2d.fromDegrees(angle.get()))));
     m_driverCtrl.b().onTrue(runOnce(() -> m_pivotSubsystem.setAngle(Rotation2d.fromDegrees(0))));
     m_operatorCtrl.povUp().onTrue(runOnce(() -> m_elevatorSubsystem.setHeight(.135)));
    m_operatorCtrl.povDown().onTrue(runOnce(() -> m_elevatorSubsystem.setHeight(0)));
   //  m_driverCtrl.leftBumper().whileTrue(runOnce(() -> m_shooterSubsystem.setVelocity(rpm.get()))).onFalse(runOnce(() -> m_shooterSubsystem.stop()));
    m_driverCtrl.y().and(ampModeSupplier).whileTrue(runOnce(() -> m_ampSubsystem.setAmpVoltage(12))).onFalse(runOnce(() -> m_ampSubsystem.stop()));
    m_operatorCtrl.leftBumper().whileTrue(runOnce(() -> m_ampSubsystem.setAmpVoltage(-10)).alongWith(runOnce(() -> m_feederSubsystem.setFeederVoltage(-10))).alongWith(runOnce(() -> m_intakeSubsystem.setIntakeVoltage(-10)))).onFalse(runOnce(() -> m_ampSubsystem.stop()).alongWith(runOnce(() -> m_feederSubsystem.stop())).alongWith(runOnce(() -> m_intakeSubsystem.stop())));
    // Stationary look and shoot with shoot when ready
  }
    public Command getAutonomousCommand() {
        /* First put the drivetrain into auto run mode, then run the auto */
        return autoChooser.getSelected();
    }

    private void newControlStyle() {
        m_controlStyle = () -> m_drive.withVelocityX(-m_driverCtrl.getLeftY() * m_MaxSpeed * invertForAlliance()) // Drive forward -Y
                .withVelocityY(-m_driverCtrl.getLeftX() * m_MaxSpeed * invertForAlliance()) // Drive left with negative X (left)
                .withRotationalRate(m_driverCtrl.rightBumper()
                .and(speakerModeSupplier).getAsBoolean()? -AprilTagLock.getR()*m_MaxAngularRate : m_driverCtrl.getRightX() * m_MaxAngularRate); // Drive counterclockwise with negative X (left)
        // Specify the desired Control Style as the Drivetrain's default command
        // Drivetrain will execute this command periodically
        m_drivetrain.setDefaultCommand(m_drivetrain.applyRequest(m_controlStyle).ignoringDisable(true));
    }
    
    public Command rumbleDriverCommand() {
        return new RunCommand(() -> rumbleDriverCtrl()).withTimeout(.2).finallyDo(() -> stopRumbleDriverCtrl());
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
