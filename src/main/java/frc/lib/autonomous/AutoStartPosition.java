package frc.lib.autonomous;

import frc.robot.Constants.ScoringGridConstants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;

public enum AutoStartPosition {
    // Path planner start poses MUST match these starting pose values!
    LoadStationMidCube("Load Station MidCube",
            new Pose2d(ScoringGridConstants.autoStartXCoordMeters,
                    ScoringGridConstants.Red.grid3CenterYMeters,
                    Rotation2d.fromDegrees(180.0)),
            new Pose2d(ScoringGridConstants.autoStartXCoordMeters,
                    ScoringGridConstants.Blue.grid6CenterYMeters,
                    Rotation2d.fromDegrees(180.0))),
    WallMidCube("Wall MidCube",
            new Pose2d(ScoringGridConstants.autoStartXCoordMeters,
                    ScoringGridConstants.Red.grid1CenterYMeters,
                    Rotation2d.fromDegrees(180.0)),
            new Pose2d(ScoringGridConstants.autoStartXCoordMeters,
                    ScoringGridConstants.Blue.grid8CenterYMeters,
                    Rotation2d.fromDegrees(180.0))),
    CenterMidCube("Center MidCube",
            new Pose2d(ScoringGridConstants.autoStartXCoordMeters,
                    ScoringGridConstants.Red.grid2CenterYMeters,
                    Rotation2d.fromDegrees(180.0)),
            new Pose2d(ScoringGridConstants.autoStartXCoordMeters,
                    ScoringGridConstants.Blue.grid7CenterYMeters,
                    Rotation2d.fromDegrees(180.0)));

    public String description;
    public Pose2d startPoseRed;
    public Pose2d startPoseBlue;

    private AutoStartPosition(String description, Pose2d poseRed, Pose2d poseBlue) {
        this.description = description;
        this.startPoseRed = poseRed;
        this.startPoseBlue = poseBlue;
    }

    public Pose2d getStartPose() {
        if (DriverStation.getAlliance() == DriverStation.Alliance.Red) {
            return this.startPoseRed;
        } else {
            return this.startPoseBlue;
        }
    }
}