// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.File;
import java.util.ArrayList;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.drive.swerve.SwerveModuleFalconFalcon;
import frc.lib.vision.LimeLight;
import frc.lib.vision.LimeLight.CoordinateSpace;
import frc.lib.vision.LimeLight.StreamMode;
import frc.robot.Constants;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

public class DrivetrainUpdated extends SubsystemBase {
  /** Creates a new DrivetrainUpdated. */

  private SwerveDrive swerveDrive;
  public double maxSpeed;
  public File swerveJsonDirectory;

 // Limelights
  public final LimeLight limeLightCameraFront;
  public final LimeLight limeLightCameraBack;
  public final LimeLight limeLightCameraLeft;
  public final LimeLight limeLightCameraRight;
  public final ArrayList<LimeLight> limelightList;
  public double limeLightBlendedLatency = 0.0;
  private double[] limelightFrontBotPose;
  private double[] limelightBackBotPose;
  private double[] limelightLeftBotPose;
  private double[] limelightRightBotPose;
  private boolean isUpdatingLimelightOdometry = true;
  private double limelightTotalArea = 0.0;
  private MedianFilter limelightXMedianFilter;
  private MedianFilter limelightYMedianFilter;
  private MedianFilter limelightAngleMedianFilter;

  private double gyroOffset = 0.0;

  private boolean simpleOdometryReset = true;
  private int odometryResetCount = 0;
  public Pose2d resetPose = Constants.DrivetrainConstants.zeroPose;

  public DrivetrainUpdated() {

    maxSpeed = Constants.DrivetrainConstants.swerveMaxSpeed;
    swerveJsonDirectory = new File(Filesystem.getDeployDirectory(),"swerve");

    SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
    try {
      swerveDrive = new SwerveParser(swerveJsonDirectory).createSwerveDrive(maxSpeed);
    } catch (Exception e) {
      throw new RuntimeException(e);
    }
    swerveDrive.setHeadingCorrection(false);
    swerveDrive.setCosineCompensator(false);
    setupPathPlanner();
  
    // Limelight
    limeLightCameraFront = new LimeLight("limelight-front");
    limelightFrontBotPose = new double[7];

    limeLightCameraBack = new LimeLight("limelight-back");
    limelightBackBotPose = new double[7];

    limeLightCameraLeft = new LimeLight("limelight-left");
    limelightLeftBotPose = new double[7];
    limeLightCameraLeft.setStreamMode(StreamMode.PiPSecondary);

    limeLightCameraRight = new LimeLight("limelight-right");
    limelightRightBotPose = new double[7];

    limelightList = new ArrayList<LimeLight>();
    limelightList.add(limeLightCameraFront);
    limelightList.add(limeLightCameraBack);
    limelightList.add(limeLightCameraLeft);
    limelightList.add(limeLightCameraRight);

    limelightXMedianFilter = new MedianFilter(1);
    limelightYMedianFilter = new MedianFilter(1);
    limelightAngleMedianFilter = new MedianFilter(1);
  }

  public Pose2d getLatestSwervePose() {
    return swerveDrive.getPose();
  }

  public void scheduleOdometryReset(Pose2d initialPose) {
    this.simpleOdometryReset = false;
    this.resetPose = initialPose;
    this.odometryResetCount++;
  }

  public ChassisSpeeds getRobotVelocity() {
    return swerveDrive.getRobotVelocity();
  }

  public void setChassisSpeeds(ChassisSpeeds chassisSpeeds) {
    swerveDrive.setChassisSpeeds(chassisSpeeds);
  }

  public void setupPathPlanner() {
    AutoBuilder.configureHolonomic(
        this::getLatestSwervePose, // Robot pose supplier
        this::scheduleOdometryReset, // Method to reset odometry (will be called if your auto has a starting pose)
        this::getRobotVelocity, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        this::setChassisSpeeds, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
        new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
          // Translation PID constants
          new PIDConstants(
            Constants.DrivetrainConstants.xyCorrectionP,
            Constants.DrivetrainConstants.xyCorrectionI,
            Constants.DrivetrainConstants.xyCorrectionD),
          // Rotation PID constants
          new PIDConstants(
            Constants.DrivetrainConstants.thetaCorrectionP,
            Constants.DrivetrainConstants.thetaCorrectionI,
            Constants.DrivetrainConstants.thetaCorrectionD),
          3.0,
          Constants.DrivetrainConstants.driveBaseRadius,
          new ReplanningConfig() // Default path replanning config. See the API for the options here
        ),
        () -> {
          // Boolean supplier that controls when the path will be mirrored for the red alliance
          // This will flip the path being followed to the red side of the field.
          // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
          var alliance = DriverStation.getAlliance();
          return alliance.isPresent() ? alliance.get() == DriverStation.Alliance.Red : false;
        },
        this // Reference to this subsystem to set requirements
    );
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public CoordinateSpace getAllianceCoordinateSpace() {
    if (!DriverStation.getAlliance().isPresent()) {
        return CoordinateSpace.Field;
    }
    if (DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Red) {
        return CoordinateSpace.Red;
    }
    return CoordinateSpace.Blue;
  }

  public void stopDrive() {
    swerveDrive.setChassisSpeeds(new ChassisSpeeds(0, 0, 0));
  }

  public double getGyroYaw() {
    // double angle = navx.getYaw() + gyroOffset;
    double angle = swerveDrive.getYaw().getDegrees() + gyroOffset;
    while (angle > 180) {
        angle -= 360;
    }
    while (angle < -180) {
        angle += 360;
    }
    return angle; // Navx is opposite sign from everything else
}

  public double getGyroOffset() {
    return gyroOffset;
  }

  public void setGyroOffset(double offset) {
    gyroOffset = offset;
  }

  public void resetGyro() {
    // navx.zeroYaw(); // This is always relative to which alliance station we are in.
    swerveDrive.zeroGyro();
    setGyroOffset(0.0);
}

public void resetGyro(double offset) {
  // Reset the gyro but set an offset
  // navx.zeroYaw();
  swerveDrive.zeroGyro();
  setGyroOffset(offset);
}



}
