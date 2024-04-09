package frc.robot.Subsystems.Peripherals;


import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.crescendo.Field;
import frc.robot.Subsystems.Drivetrain.CommandSwerveDrivetrain;
import frc.robot.Subsystems.Drivetrain.Telemetry;
import frc.robot.Util.vision.Limelight;
import frc.robot.Util.vision.Limelight.PhysicalConfig;
import frc.robot.Util.vision.LimelightHelpers.RawFiducial;
import java.text.DecimalFormat;
//import org.littletonrobotics.junction.AutoLogOutput;

public class Vision extends SubsystemBase {
     CommandSwerveDrivetrain drivetrain;
     Telemetry telemetry;
    public static final class VisionConfig {
        /* Limelight Configuration */
        public static final String REAR_LL = "limelight-rear";
        // These don't seem to actually be setting at the limelight, had to manually adjust them
        public static final PhysicalConfig REAR_CONFIG =
                new PhysicalConfig().withTranslation(-0.296, 0, 0.226).withRotation(0, 50, 180);

        public static final String SPEAKER_LL = "limelight-aim";

        public static final PhysicalConfig SPEAKER_CONFIG =
                new PhysicalConfig().withTranslation(-0.085, 0, 0.636).withRotation(0, 15, 0);

        // public static final String LEFT_LL = "limelight-left";

        // public static final PhysicalConfig LEFT_CONFIG =
        //         new PhysicalConfig().withTranslation(0, 0, 0).withRotation(0, 0, 0); // TODO: input

        // public static final String RIGHT_LL = "limelight-right";

        // public static final PhysicalConfig RIGHT_CONFIG =
        //         new PhysicalConfig().withTranslation(0, 0, 0).withRotation(0, 0, 0); // TODO: input

        // Limelight 2+
        // public static final PhysicalConfig SPEAKER_CONFIG =
        //         new PhysicalConfig().withTranslation(-0.085, 0, 0.636).withRotation(0, 15, 0);

        /* Pipeline config */
        public static final int rearDetectorPipeline = 0;
        public static final int speakerDetectorPipeline = 0;
        public static final int leftDetectorPipeline = 0;
        public static final int rightDetectorPipeline = 0;

        /* AprilTag Heights (meters) */
        public static final double speakerTagHeight = 1.45;
        public static final int speakerTagID = 4;

        /* Pose Estimation Constants */
        public static final double VISION_REJECT_DISTANCE = 1.8; // 2.3;

        // Increase these numbers to trust global measurements from vision less.
        public static double VISION_STD_DEV_X = 0.5;
        public static double VISION_STD_DEV_Y = 0.5;
        public static double VISION_STD_DEV_THETA = 99999999;

        public static final Matrix<N3, N1> visionStdMatrix =
                VecBuilder.fill(VISION_STD_DEV_X, VISION_STD_DEV_Y, VISION_STD_DEV_THETA);
    }

    public record AimingParameters(
            Rotation2d driveHeading,
            Rotation2d armAngle,
            double effectiveDistance,
            double driveFeedVelocity) {}

    /* Limelights */
    public final Limelight rearLL =
            new Limelight(
                    VisionConfig.REAR_LL,
                    VisionConfig.rearDetectorPipeline,
                    VisionConfig.REAR_CONFIG);
    public final LimelightLogger rearLogger = new LimelightLogger("Rear", rearLL);
    public final Limelight speakerLL =
            new Limelight(
                    VisionConfig.SPEAKER_LL,
                    VisionConfig.speakerDetectorPipeline,
                    VisionConfig.SPEAKER_CONFIG);
    public final LimelightLogger speakerLogger = new LimelightLogger("Front", speakerLL);
    // public final Limelight leftLL =
    //         new Limelight(
    //                 VisionConfig.LEFT_LL,
    //                 VisionConfig.leftDetectorPipeline,
    //                 VisionConfig.LEFT_CONFIG);
    // public final LimelightLogger leftLogger = new LimelightLogger("Left", leftLL);
    // public final Limelight rightLL =
    //         new Limelight(
    //                 VisionConfig.RIGHT_LL,
    //                 VisionConfig.rightDetectorPipeline,
    //                 VisionConfig.RIGHT_CONFIG);
    // public final LimelightLogger rightLogger = new LimelightLogger("Right", rightLL);
    public final Limelight[] limelights = {speakerLL, rearLL};

    private final DecimalFormat df = new DecimalFormat();

   //@AutoLogOutput(key = "Vision/a_Integrating")
    public static boolean isIntegrating = false;

    private boolean isAiming = false;

    // /** Cached latest aiming parameters. Calculated in {@code getAimingParameters()} */
    // private AimingParameters latestParameters = null;

    // private static final double poseBufferSizeSeconds = 2.0;
    // private final TimeInterpolatableBuffer<Pose2d> poseBuffer =
    //         TimeInterpolatableBuffer.createBuffer(poseBufferSizeSeconds);

    public Vision() {
        setName("Vision");

        // logging
        df.setMaximumFractionDigits(2);

        /* Configure Limelight Settings Here */
        for (Limelight limelight : limelights) {
            limelight.setLEDMode(false);
        }
    }

    @Override
    public void periodic() {
        try {
            isIntegrating = false;
            // Will NOT run in auto
            if (DriverStation.isTeleopEnabled()) {
                // force pose to be vision
                Pose2d estimatedPose = drivetrain.getState().Pose;
                if ((estimatedPose.getX() <= 0.1 || estimatedPose.getY() <= 0.1)) {
                    // forcePoseToVision();
                }

                // if the front camera sees tag and we are aiming, only use that camera
                if (isAiming && speakerLL.targetInView()) {
                    for (Limelight limelight : limelights) {
                        if (limelight.CAMERA_NAME == speakerLL.CAMERA_NAME) {
                            filterAndAddVisionMeasurment(limelight);
                        } else {
                            limelight.sendInvalidStatus("speaker only rejection");
                        }
                        isIntegrating |= limelight.isIntegrating;
                    }
                } else {
                    // choose LL with best view of tags and integrate from only that camera
                    Limelight bestLimelight = getBestLimelight();
                    for (Limelight limelight : limelights) {
                        if (limelight.CAMERA_NAME == bestLimelight.CAMERA_NAME) {
                            filterAndAddVisionMeasurment(bestLimelight);
                        } else {
                            limelight.sendInvalidStatus("not best rejection");
                        }
                        isIntegrating |= limelight.isIntegrating;
                    }
                }
            }
        } catch (Exception e) {
           // telemetry.print("Vision pose not present but tried to access it");
        }
    }

    private void filterAndAddVisionMeasurment(Limelight ll) {
        double xyStds = 1000;
        double degStds = 1000;

        // integrate vision
        if (ll.targetInView()) {
            boolean multiTags = ll.multipleTagsInView();
            double timeStamp = ll.getVisionPoseTimestamp();
            double targetSize = ll.getTargetSize();
            Pose3d botpose3D = ll.getRawPose3d();
            Pose2d botpose = botpose3D.toPose2d();
            RawFiducial[] tags = ll.getRawFiducial();
            ChassisSpeeds robotSpeed = drivetrain.getFieldRelativeChassisSpeeds();

            // distance from current pose to vision estimated pose
            double poseDifference =
                    drivetrain.getState().Pose.getTranslation().getDistance(botpose.getTranslation());

            /* rejections */
            // reject pose if individual tag ambiguity is too high
            ll.tagStatus = "";
            for (RawFiducial tag : tags) {
                ll.tagStatus += "Tag " + tag.id + ": " + tag.ambiguity;
                if (tag.ambiguity > 0.5) {
                    ll.sendInvalidStatus("ambiguity rejection");
                    return;
                }
            }
            if (Field.poseOutOfField(botpose3D)) {
                // reject if pose is out of the field
                ll.sendInvalidStatus("bound rejection");
                return;
            } else if (Math.abs(robotSpeed.omegaRadiansPerSecond) >= 0.5) {
                // reject if we are rotating more than 0.5 rad/s
                ll.sendInvalidStatus("rotation rejection");
            } else if (Math.abs(botpose3D.getZ()) > 0.25) {
                // reject if pose is .25 meters in the air
                ll.sendInvalidStatus("height rejection");
                return;
            } else if (Math.abs(botpose3D.getRotation().getX()) > 5
                    || Math.abs(botpose3D.getRotation().getY()) > 5) {
                // reject if pose is 5 degrees titled in roll or pitch
                ll.sendInvalidStatus("roll/pitch rejection");
            }
            //  else if (poseDifference < Units.inchesToMeters(3)) {
            //     // reject if pose is very close to robot pose
            //     isPresent = false;
            //     ll.logStatus = "proximity rejection";
            //     return;
            // }
            /* integrations */
            // if almost stationary and extremely close to tag
            else if (robotSpeed.vxMetersPerSecond + robotSpeed.vyMetersPerSecond <= 0.2
                    && targetSize > 0.4) {
                ll.sendValidStatus("Stationary close integration");
                xyStds = 0.1;
                degStds = 0.1;
            } else if (multiTags && targetSize > 0.05) {
                ll.sendValidStatus("Multi integration");
                xyStds = 0.5;
                degStds = 8;
                if (targetSize > 0.09) {
                    ll.sendValidStatus("Strong Multi integration");
                    xyStds = 0.1;
                    degStds = 0.1;
                }
            } else if (targetSize > 0.8 && poseDifference < 0.5) {
                ll.sendValidStatus("Close integration");
                xyStds = 1.0;
                degStds = 16;
            } else if (targetSize > 0.1 && poseDifference < 0.3) {
                ll.sendValidStatus("Proximity integration");
                xyStds = 2.0;
                degStds = 999999;
            } else {
                ll.sendInvalidStatus(
                        "catch rejection: "
                               // + RobotTelemetry.truncatedDouble(poseDifference)
                                + " poseDiff");
                return;
            }

            // track STDs
            VisionConfig.VISION_STD_DEV_X = xyStds;
            VisionConfig.VISION_STD_DEV_Y = xyStds;
            VisionConfig.VISION_STD_DEV_THETA = degStds;

            drivetrain.setVisionMeasurementStdDevs(
                    VecBuilder.fill(
                            VisionConfig.VISION_STD_DEV_X,
                            VisionConfig.VISION_STD_DEV_Y,
                            VisionConfig.VISION_STD_DEV_THETA));
            drivetrain.addVisionMeasurement(botpose, timeStamp);
        } else {
            ll.sendInvalidStatus("no tag found rejection");
        }
    }

    /**
     * REQUIRES ACCURATE POSE ESTIMATION. Uses trigonometric functions to calculate the angle
     * between the robot heading and the angle required to face the speaker center.
     *
     * @return angle between robot heading and speaker in degrees
     */
    public double getThetaToSpeaker() {
        // Translation2d speaker =
        //         Field.flipXifRed(Field.Speaker.centerSpeakerOpening).toTranslation2d();
        Translation2d speaker =
                Field.flipXifRed(Field.Speaker.centerSpeakerPose)
                        .getTranslation(); // getAdjustedSpeakerPos();
        Translation2d robotXY = drivetrain.getState().Pose.getTranslation();
        double angleBetweenRobotAndSpeaker =
                MathUtil.angleModulus(speaker.minus(robotXY).getAngle().getRadians());

        return angleBetweenRobotAndSpeaker;
    }

    public double getAdjustedThetaToSpeaker() {
        Translation2d speaker = getAdjustedSpeakerPos();
        Translation2d robotXY = drivetrain.getState().Pose.getTranslation();
        double angleBetweenRobotAndSpeaker =
                MathUtil.angleModulus(speaker.minus(robotXY).getAngle().getRadians());

        return angleBetweenRobotAndSpeaker;
    }

    /** Returns the distance from the speaker in meters, adjusted for the robot's movement. */
   //@AutoLogOutput(key = "Vision/SpeakerDistance")
    public double getSpeakerDistance() {
        double poseDistance =
                drivetrain.getState().Pose.getTranslation().getDistance(getAdjustedSpeakerPos());
        double tagDistance = getDistanceToCenterSpeakerTagFromRobot();
        if (tagDistance != -1) {
            return poseDistance; // tagDistance;
        }
        return poseDistance;
    }

   //@AutoLogOutput(key = "Vision/SpeakerYDistance")
    public double getSpeakerYDelta() {
        return drivetrain.getState().Pose.getTranslation().getY() - getAdjustedSpeakerPos().getY();
    }

    public Translation2d getAdjustedSpeakerPos() {
        return getAdjustedTargetPos(
                new Translation2d(0, Field.Speaker.centerSpeakerOpening.toTranslation2d().getY()));
    }

    // Returns distance to the center of the speaker tag from the robot or -1 if not found
    public double getDistanceToCenterSpeakerTagFromRobot() {
        RawFiducial[] tags = speakerLL.getRawFiducial();
        int speakerTagID = 7; // Blue Speaker Tag
        if (Field.isRed()) {
            speakerTagID = 4; // Red Speaker Tag
        }

        for (RawFiducial tag : tags) {
            if (tag.id == speakerTagID) {
                return tag.distToRobot;
            }
        }

        return -1;
    }

    /**
     * Gets a field-relative position for the shot to the speaker the robot should take, adjusted
     * for the robot's movement.
     *
     * @return A {@link Translation2d} representing a field relative position in meters.
     */
    public Translation2d getAdjustedTargetPos(Translation2d targetPose) {
        // double NORM_FUDGE = 0.075;
        // double tunableNoteVelocity = 1;
        // double tunableNormFudge = 0;
        // double tunableStrafeFudge = 1;
        double tunableSpeakerYFudge = 0.0;
        double tunableSpeakerXFudge = 0.0;
        double spinYFudge = 0.05;

        targetPose = Field.flipXifRed(targetPose);
        // Translation2d robotPos = drivetrain.getState().Pose.getTranslation();
        // ChassisSpeeds robotVel = drivetrain.getFieldRelativeChassisSpeeds(); // TODO: change

        // double distance = robotPos.getDistance(targetPose);
        // double normFactor =
        //         Math.hypot(robotVel.vxMetersPerSecond, robotVel.vyMetersPerSecond) < NORM_FUDGE
        //                 ? 0.0
        //                 : Math.abs(
        //                         MathUtil.angleModulus(
        //                                         robotPos.minus(targetPose).getAngle().getRadians()
        //                                                 - Math.atan2(
        //                                                         robotVel.vyMetersPerSecond,
        //                                                         robotVel.vxMetersPerSecond))
        //                                 / Math.PI);

        double x =
                targetPose.getX() + (Field.isBlue() ? tunableSpeakerXFudge : -tunableSpeakerXFudge);
        // - (robotVel.vxMetersPerSecond
        // * (distance / tunableNoteVelocity)
        //      * (1.0 - (tunableNormFudge * normFactor)));
        double y =
                targetPose.getY()
                        + (Field.isBlue() ? -spinYFudge : spinYFudge)
                        + tunableSpeakerYFudge;
        // - (robotVel.vyMetersPerSecond
        // * (distance / tunableNoteVelocity)
        //       * tunableStrafeFudge);

        return new Translation2d(x, y);
    }

    public double getAdjustedThetaToFeeder() {
        Translation2d feeder = getAdjustedFeederPos();
        Translation2d robotXY = drivetrain.getState().Pose.getTranslation();
        double angleBetweenRobotAndFeeder =
                MathUtil.angleModulus(feeder.minus(robotXY).getAngle().getRadians());

        return angleBetweenRobotAndFeeder;
    }

    /** Returns the distance from the feed position in meters, adjusted for the robot's movement. */
   //@AutoLogOutput(key = "Vision/FeedDistance")
    public double getFeedDistance() {
        return drivetrain.getState().Pose.getTranslation().getDistance(getAdjustedFeederPos());
    }

    public Translation2d getAdjustedFeederPos() {
        Translation2d originalLocation = Field.StagingLocations.spikeTranslations[2];
        Translation2d newLocation;
        if (Field.isBlue()) {
            newLocation =
                    new Translation2d(originalLocation.getX() - 2.0, originalLocation.getY() + 0.5);
        } else {
            newLocation =
                    new Translation2d(originalLocation.getX() - 2.0, originalLocation.getY() + 1);
        }
        return getAdjustedTargetPos(newLocation);
    }

    // /**
    //  * Set robot pose to vision pose ONLY USING SPEAKER LL regardless of validity. Does not reset
    //  * rotation.
    //  */
    // public void forcePoseToVision() {
    //     // TODO: add more fallback logic here
    //     drivetrain.resetPose(
    //             drivetrain.convertPoseWithGyro(speakerLL.getRawPose3d().toPose2d()));
    // }

    // /** Set robot pose to vision pose only if LL has good tag reading */
    // public void resetPoseToVision() {
    //     for (Limelight limelight : limelights) {
    //         if (limelight.hasAccuratePose()) {
    //             drivetrain.resetPose(limelight.getRawPose3d().toPose2d());
    //             break;
    //         }
    //     }
    // }

    public Limelight getBestLimelight() {
        Limelight bestLimelight = speakerLL;
        double bestScore = 0;
        for (Limelight limelight : limelights) {
            double score = 0;
            // prefer LL with most tags, when equal tag count, prefer LL closer to tags
            score += limelight.getTagCountInView();
            score += limelight.getTargetSize();

            if (score > bestScore) {
                bestScore = score;
                bestLimelight = limelight;
            }
        }
        return bestLimelight;
    }

    // //6328-2024 Stdev adjustment stuff
    // public void addVisionObservation(VisionObservation observation) {
    //     latestParameters = null;
    //     // If measurement is old enough to be outside the pose buffer's timespan, skip.
    //     try {
    //     if (poseBuffer.getInternalBuffer().lastKey() - poseBufferSizeSeconds
    //         > observation.timestamp()) {
    //         return;
    //     }
    //     } catch (NoSuchElementException ex) {
    //     return;
    //     }
    //     // Get odometry based pose at timestamp
    //     var sample = poseBuffer.getSample(observation.timestamp());
    //     if (sample.isEmpty()) {
    //     // exit if not there
    //     return;
    //     }

    //     // sample --> odometryPose transform and backwards of that
    //     var sampleToOdometryTransform = new Transform2d(sample.get(), odometryPose);
    //     var odometryToSampleTransform = new Transform2d(odometryPose, sample.get());
    //     // get old estimate by applying odometryToSample Transform
    //     Pose2d estimateAtTime = estimatedPose.plus(odometryToSampleTransform);

    //     // Calculate 3 x 3 vision matrix
    //     var r = new double[3];
    //     for (int i = 0; i < 3; ++i) {
    //     r[i] = observation.stdDevs().get(i, 0) * observation.stdDevs().get(i, 0);
    //     }
    //     // Solve for closed form Kalman gain for continuous Kalman filter with A = 0
    //     // and C = I. See wpimath/algorithms.md.
    //     Matrix<N3, N3> visionK = new Matrix<>(Nat.N3(), Nat.N3());
    //     for (int row = 0; row < 3; ++row) {
    //     double stdDev = qStdDevs.get(row, 0);
    //     if (stdDev == 0.0) {
    //         visionK.set(row, row, 0.0);
    //     } else {
    //         visionK.set(row, row, stdDev / (stdDev + Math.sqrt(stdDev * r[row])));
    //     }
    //     }
    //     // difference between estimate and vision pose
    //     Transform2d transform = new Transform2d(estimateAtTime, observation.visionPose());
    //     // scale transform by visionK
    //     var kTimesTransform =
    //         visionK.times(
    //             VecBuilder.fill(
    //                 transform.getX(), transform.getY(), transform.getRotation().getRadians()));
    //     Transform2d scaledTransform =
    //         new Transform2d(
    //             kTimesTransform.get(0, 0),
    //             kTimesTransform.get(1, 0),
    //             Rotation2d.fromRadians(kTimesTransform.get(2, 0)));

    //     // Recalculate current estimate by applying scaled transform to old estimate
    //     // then replaying odometry data
    //     estimatedPose = estimateAtTime.plus(scaledTransform).plus(sampleToOdometryTransform);
    // }

    /**
     * If at least one LL has an accurate pose
     *
     * @return
     */
    public boolean hasAccuratePose() {
        for (Limelight limelight : limelights) {
            if (limelight.hasAccuratePose()) return true;
        }
        return false;
    }

    /** Change both LL pipelines to the same pipeline */
    public void setLimelightPipelines(int pipeline) {
        for (Limelight limelight : limelights) {
            limelight.setLimelightPipeline(pipeline);
        }
    }

    /** Set both LLs to blink */
    public Command blinkLimelights() {
        return startEnd(
                        () -> {
                            for (Limelight limelight : limelights) {
                                limelight.blinkLEDs();
                            }
                        },
                        () -> {
                            for (Limelight limelight : limelights) {
                                limelight.setLEDMode(false);
                            }
                        })
                .withName("Vision.blinkLimelights");
    }

    public void setAiming() {
        isAiming = true;
    }

    public void setNotAiming() {
        isAiming = false;
    }

    /** Logging */

    // can't use autologoutput in library and avoid repetitive loggers
    public static class LimelightLogger {
        private final Limelight limelight;
        @SuppressWarnings("unused")
        private String name;

        public LimelightLogger(String name, Limelight limelight) {
            this.limelight = limelight;
            this.name = name;
        }

       //@AutoLogOutput(key = "Vision/{name}/ConnectionStatus")
        public boolean getCameraConnection() {
            return limelight.isCameraConnected();
        }

       //@AutoLogOutput(key = "Vision/{name}/Integrating")
        public boolean getIntegratingStatus() {
            return limelight.isIntegrating;
        }

       //@AutoLogOutput(key = "Vision/{name}/LogStatus")
        public String getLogStatus() {
            return limelight.logStatus;
        }

       //@AutoLogOutput(key = "Vision/{name}/TagStatus")
        public String getTagStatus() {
            return limelight.tagStatus;
        }

       //@AutoLogOutput(key = "Vision/{name}/Pose")
        public Pose2d getPose() {
            return limelight.getRawPose3d().toPose2d();
        }

       //@AutoLogOutput(key = "Vision/{name}/PoseX")
        public double getPoseX() {
            return getPose().getX();
        }

       //@AutoLogOutput(key = "Vision/{name}/PoseY")
        public double getPoseY() {
            return getPose().getY();
        }

       //@AutoLogOutput(key = "Vision/{name}/TagCount")
        public double getTagCount() {
            return limelight.getTagCountInView();
        }

       //@AutoLogOutput(key = "Vision/{name}/TargetSize")
        public double getTargetSize() {
            return limelight.getTargetSize();
        }
    }

    public static class CommandConfig {
        public double kp;
        public double tolerance;
        public double maxOutput;
        public double error;
        public int pipelineIndex;
        public Limelight limelight;
        /* For Drive-To commands */
        public CommandConfig alignCommand;
        public double verticalSetpoint; // numbers get small as the cone gets closer
        public double verticalMaxView;

        public void configKp(double kp) {
            this.kp = kp;
        }

        public void configTolerance(double tolerance) {
            this.tolerance = tolerance;
        }

        public void configMaxOutput(double maxOutput) {
            this.maxOutput = maxOutput;
        }

        public void configError(double error) {
            this.error = error;
        }

        public void configPipelineIndex(int pipelineIndex) {
            this.pipelineIndex = pipelineIndex;
        }

        public void configLimelight(Limelight limelight) {
            this.limelight = limelight;
        }

        public void configVerticalSetpoint(double verticalSetpoint) {
            this.verticalSetpoint = verticalSetpoint;
        }

        public void configVerticalMaxView(double verticalMaxView) {
            this.verticalMaxView = verticalMaxView;
        }

        public void configAlignCommand(CommandConfig alignCommand) {
            this.alignCommand = alignCommand;
        }

        public CommandConfig() {}
    }
}
