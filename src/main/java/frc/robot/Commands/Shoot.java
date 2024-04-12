package frc.robot.Commands;

import java.util.function.BooleanSupplier;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Setpoints;
import frc.robot.Subsystems.Drivetrain.CommandSwerveDrivetrain;
import frc.robot.Subsystems.Feeder.FeederSubsystem;
import frc.robot.Subsystems.Intake.IntakeSubsystem;
import frc.robot.Subsystems.Pivot.PivotSubsystem;
import frc.robot.Subsystems.Shooter.ShooterSubsystem;
import frc.robot.Util.FieldCentricAiming;
import frc.robot.Util.TunableNumber;
import frc.robot.interpolation.ShooterInterpolation;

public class Shoot extends Command {

    CommandSwerveDrivetrain m_drivetrain;
    IntakeSubsystem m_intake;
    FeederSubsystem m_feeder;
    PivotSubsystem m_pivot;
    ShooterSubsystem m_shooter;
    FieldCentricAiming m_FieldCentricAiming;

    Pose2d currentRobotPose;
    Translation2d currentRobotTranslation;
    Rotation2d currentAngleToSpeaker;

    Pose2d futureRobotPose2d;
    Translation2d futureRobotTranslation;
    Rotation2d futureAngleToSpeaker;

    Rotation2d m_angle;
    double m_rpm;

    ChassisSpeeds speeds;
    Translation2d moveDelta;
    Translation2d armDelta;

    /**
     * The calculated the time until the note leaves based on the constant and time
     * since button press
     */
    Double timeUntilShot;
    BooleanSupplier m_isShooting;

    Double correctedDistance;
    Rotation2d correctedRotation;

    Rotation2d lockedRotation;
    double lockedDistance;

    TunableNumber timeToShoot = new TunableNumber("Smart timeToShoot", .2);
    TunableNumber timeToBeReady = new TunableNumber("Smart timeToBeReady", .5);

    boolean m_isShootOnTheMove;
    Timer shotTimer;
    boolean timerIsRunning = false;

    boolean m_isFinished = false;

    /** Creates a new smartShootOnMove. */
    public Shoot(CommandSwerveDrivetrain drivetrain, FeederSubsystem feeder,
            PivotSubsystem pivot, ShooterSubsystem shooter, boolean isShootOnTheMove) {

        m_drivetrain = drivetrain;
        m_feeder = feeder;
        m_pivot = pivot;
        m_shooter = shooter;
        m_isShootOnTheMove = isShootOnTheMove;
        m_FieldCentricAiming = new FieldCentricAiming();
        shotTimer = new Timer();

        addRequirements(m_pivot, m_feeder, m_shooter);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        System.out.println("SCHEDULED");

        m_isFinished = false;
        timerIsRunning = false;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        currentRobotTranslation = m_drivetrain.getState().Pose.getTranslation();
        currentAngleToSpeaker = m_FieldCentricAiming.getAngleToSpeaker(currentRobotTranslation);
        SmartDashboard.putNumber("x",currentAngleToSpeaker.getDegrees());
        // Get current drivetrain velocities in field relative terms
        speeds = m_drivetrain.getFieldRelativeChassisSpeeds();

        // Calculate change in x/y distance due to time and velocity
        moveDelta = new Translation2d(timeToBeReady.get() * (speeds.vxMetersPerSecond),
                timeToBeReady.get() * (speeds.vyMetersPerSecond));

        futureRobotTranslation = currentRobotTranslation.plus(moveDelta);
        futureAngleToSpeaker = m_FieldCentricAiming.getAngleToSpeaker(futureRobotTranslation);

        correctedDistance = m_FieldCentricAiming.getDistToSpeaker(futureRobotTranslation);
        correctedRotation = futureAngleToSpeaker;

        if (timerIsRunning) {
            System.out.println(shotTimer.get());
            if (shotTimer.hasElapsed(timeToBeReady.get() / 2)) {
                if (!m_shooter.isAtSetpoint() || !m_pivot.isAtSetpoint() || m_drivetrain.isRotatingFast()
                        || (!m_isShootOnTheMove && m_drivetrain.isMoving())) {
                    System.out.println("WONT BE READY, RESTARTING SHOT");
                    shotTimer.stop();
                    shotTimer.reset();
                    timerIsRunning = false;
                    if (!m_shooter.isAtSetpoint()) {
                        System.out.println("Shooter is not at speed");
                    }
                    if (m_drivetrain.isRotatingFast()) {
                        System.out.println("Drivetrain is rotating too fast");
                    }
                    if (!m_pivot.isAtSetpoint()) {
                        System.out.println("Arm is not at setpoint");
                    }
                    if (!m_isShootOnTheMove && m_drivetrain.isMoving()) {
                        System.out.println("Drivetrain is moving during static shot");
                    }
                }

            }
            if (shotTimer.hasElapsed(timeToBeReady.get() - timeToShoot.get()) && !m_feeder.isFeederRunning()) {
                System.out.println("Starting stage");
                m_feeder.setFeederVoltage(Setpoints.scoringFeedVolts);
                SmartDashboard.putNumber("Angle error at t=0",
                        currentAngleToSpeaker.minus(m_drivetrain.getState().Pose.getRotation()).getDegrees());
                SmartDashboard.putNumber("Angle error at t=0",
                        currentAngleToSpeaker.minus(m_drivetrain.getState().Pose.getRotation()).getDegrees());
            }

            if (shotTimer.hasElapsed(timeToBeReady.get())) {
                System.out.println("Shot should be complete now");

                m_isFinished = true;
            }

        } else {
            if (m_isShootOnTheMove && correctedDistance <= 5) {
                System.out.println("STARTING READYUP TIMER FOR DYNAMIC SHOT ");
                shotTimer.start();
                timerIsRunning = true;
            } else if (!m_isShootOnTheMove && correctedDistance <= 4.5) {
                System.out.println("STARTING READYUP TIMER FOR STATIC SHOT ");
                shotTimer.start();
                timerIsRunning = true;
            }
            lockedRotation = correctedRotation;
            lockedDistance = correctedDistance;
        }

        m_angle = ShooterInterpolation.calculatePivotAngle(lockedDistance);
        m_rpm = ShooterInterpolation.calculateShooterRPM(lockedDistance);

        m_pivot.setAngle(m_angle);
        m_shooter.setVelocity(m_rpm);

        m_drivetrain.setVelocityOffset(lockedRotation, lockedDistance); // Pass the offsets to the drivetrain
        m_drivetrain.setOverrideAngle(lockedRotation);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        shotTimer.stop();
        shotTimer.reset();
        m_shooter.stop();
        m_feeder.stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return m_isFinished;
    }
}

