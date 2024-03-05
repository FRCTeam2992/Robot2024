package frc.lib.autonomous;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.AutoConstants;

public enum AutoStartPosition {
    
    // Path planner start poses MUST match these starting pose values!
    Right("Right",
    new Pose2d(AutoConstants.StartLocations.rightX,
    AutoConstants.StartLocations.redRightY,
    Rotation2d.fromDegrees(AutoConstants.StartLocations.rightAngle)),
    new Pose2d(AutoConstants.StartLocations.rightX,
    AutoConstants.StartLocations.blueRightY,
    Rotation2d.fromDegrees(AutoConstants.St
                            artLocations.rightAngle))),
    Center("Center",
    new Pose2d(AutoConstants.StartLocations.centerX,
    AutoConstants.StartLocations.redCenterY,
    Rotation2d.fromDegrees(0.0)),
    new Pose2d(AutoConstants.StartLocations.centerX,
    AutoConstants.StartLocations.blueCenterY,
    Rotation2d.fromDegrees(0.0
                            ))),
    Left("Left",
    new Pose2d(AutoConstants.StartLocations.leftX,
    AutoConstants.StartLocations.redLeftY,
    Rotation2d.fromDegrees(AutoConstants.StartLocations.leftAngle)),
    new Pose2d(AutoConstants.StartLocations.leftX,
    AutoConstants.StartLocations.blueLeftY,
    Rotation2d.fromDegrees(AutoConstants.StartLocations.leftAngle)));
    
    public String description;
    public Pose2d startPoseRed;
    public Pose2d startPoseBlue;
    
    private AutoStartPosition(String description, Pose2d poseRed, Pose2d poseBlue) {
        this.description = description;
        this.startPoseRed = poseRed;
        this.startPoseBlue = poseBlue;
    }
    
    public Pose2d getStartPose() {
        if (DriverStation.getAlliance().orElse(DriverStation.Alliance.Red) == DriverStation.Alliance.Red) {
            return this.startPoseRed;
        } else {
            return this.startPoseBlue;
        }
    }
}