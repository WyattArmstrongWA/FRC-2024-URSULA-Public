package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Setpoints;
import frc.robot.Subsystems.Feeder.FeederSubsystem;
import frc.robot.Subsystems.Pivot.PivotSubsystem;
import frc.robot.Subsystems.Shooter.ShooterSubsystem;
import frc.robot.Vision.Limelight;
import frc.robot.interpolation.ShooterInterpolation;

public class Shoot extends Command {
    private ShooterSubsystem shooter;
    private PivotSubsystem pivot;
    private Limelight limelight;
    private FeederSubsystem feeder;
    private Boolean auto;
    


  /** Creates a new Shoot. */
  public Shoot(ShooterSubsystem shooter, PivotSubsystem pivot, Limelight limelight, FeederSubsystem feeder, Boolean auto) {

    this.shooter = shooter;
    this.pivot = pivot;
    this.limelight = limelight;
    this.feeder = feeder;
    this.auto = auto;


    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter, pivot, limelight, feeder);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }


  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // Translation2d target = ScoringConstants.shootWhileMoving
    // ?
    // PoseEstimation.calculateVirtualSpeakerOffset(FieldUtil.getAllianceSpeakerPosition())
    // : FieldUtil.getAllianceSpeakerPosition();

    double targetDistance = limelight.getDistance();

    
  

    pivot.setAngle(ShooterInterpolation.calculatePivotAngle(targetDistance));

    // double[] speeds = targetDistance < ScoringConstants.flywheelDistanceCutoff ? ScoringConstants.shooterSetpointClose
    //     : ScoringConstants.shooterSetpointFar;
    shooter.setVelocity(ShooterInterpolation.calculateShooterRPM(targetDistance));

    if ((shooter.isAtSetpoint() && pivot.isAtSetpoint())) {
      feeder.setFeederVoltage(Setpoints.scoringFeedVolts);
    } else {
      feeder.setFeederVoltage(0);
    }

    // SmartDashboard.putNumber("TargetAngle", SpeakerShotRegression.calculateWristAngle(targetDistance).getDegrees());
    // SmartDashboard.putNumber("TargetDistance", targetDistance);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.stop();
    feeder.stop();
    pivot.stow();

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(auto==true) {
      return !feeder.isNotePresentTOF();
  }
    return false;
  }
}