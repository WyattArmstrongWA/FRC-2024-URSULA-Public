// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Util;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants;


public class FieldCentricAiming {
    private Translation2d defaultPosition = new Translation2d(0,0);
    private Translation2d speakerPosition = defaultPosition;
    
    public FieldCentricAiming() {
        try {
            getSpeakerPos();
        } catch (Exception e) {
            System.out.println(e);
        }
        
    }

    public Translation2d getSpeakerPos() {
        if (speakerPosition == defaultPosition) {
            if (DriverStation.getAlliance() != null && !DriverStation.getAlliance().isEmpty()) {
                speakerPosition = (DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) ? Constants.FieldConstants.BLUE_SPEAKER : Constants.FieldConstants.RED_SPEAKER;
            }
        }
        return speakerPosition;
    }

    public double getDistToSpeaker(Translation2d robotPose) {
        return getSpeakerPos().getDistance(robotPose);
    }

    public Rotation2d getAngleToSpeaker(Translation2d pose) {
        return getSpeakerPos().minus(pose).getAngle();
    }

}
