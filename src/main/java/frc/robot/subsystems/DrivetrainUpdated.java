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
import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

  public Pose2d latestSwervePose = new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0.0));
  public Pose2d latestVisionPose = new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0.0));
  public boolean latestVisionPoseValid = false; // Do we have a current vision sighting?
  private double gyroOffset = 0.0;

// Swerve Drive Kinematics
    public final SwerveDriveKinematics swerveDriveKinematics;

    // Swerve Drive Odometrys
    public final SwerveDrivePoseEstimator swerveDrivePoseEstimator;
    public SwerveModulePosition[] swerveDriveModulePositions = {
            new SwerveModulePosition(),
            new SwerveModulePosition(),
            new SwerveModulePosition(),
            new SwerveModulePosition()
    };
  
  public Transform2d moved;

  // State Variables
  private boolean inSlowMode = true;
  private boolean doFieldOrient = true;
  private boolean autoRotate = false;
  private boolean loadingMode = false;
  private boolean useLimelightOdometryUpdates = false;
  private int lastOdometryResetCount = -1;
  private boolean simpleOdometryReset = true;
  private int odometryResetCount = 0;
  public Pose2d resetPose = Constants.DrivetrainConstants.zeroPose;

  private boolean odomReadingTesting = false;

  private int dashboardCounter = 0;

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

    swerveDriveKinematics = new SwerveDriveKinematics(
                Constants.DrivetrainConstants.frontLeftLocation,
                Constants.DrivetrainConstants.frontRightLocation,
                Constants.DrivetrainConstants.rearLeftLocation,
                Constants.DrivetrainConstants.rearRightLocation);

    swerveDrivePoseEstimator = new SwerveDrivePoseEstimator(
                swerveDriveKinematics,
                Rotation2d.fromDegrees(-getGyroYaw()),
                swerveDriveModulePositions,
                new Pose2d(0.0, 0.0, new Rotation2d()),
                // State measurement standard deviations. X, Y, theta.
                MatBuilder.fill(Nat.N3(), Nat.N1(), 0.002, 0.002, 0.01),
                // Global measurement standard deviations. X, Y, and theta.
                MatBuilder.fill(Nat.N3(), Nat.N1(), 0.01, 0.01, .9999));
  }

  public Pose2d getLatestSwervePose() {
    return swerveDrive.getPose();
  }

  public int getOdometryResetCount() {
    return this.odometryResetCount;
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
    if (!Constants.DrivetrainConstants.odometryThread) {
            if (getOdometryResetCount() > lastOdometryResetCount) {
                // Reset odometry if needed
                resetOdometry();
                lastOdometryResetCount = getOdometryResetCount();
            } else {
                if (useLimeLightForOdometry()) {
                    calculateBlendedVisionPose();
                    if (latestVisionPoseValid) {
                        swerveDrivePoseEstimator.addVisionMeasurement(
                                latestVisionPose, Timer.getFPGATimestamp() - limeLightBlendedLatency / 1000);
                    }
                }

                // Update odometry via wheel encoders
                // swerveDriveModulePositions[0] = frontLeftModule.getPosition();
                // swerveDriveModulePositions[1] = frontRightModule.getPosition();
                // swerveDriveModulePositions[2] = rearLeftModule.getPosition();
                // swerveDriveModulePositions[3] = rearRightModule.getPosition();
                swerveDriveModulePositions[0] = swerveDrive.getModulePositions()[0];
                swerveDriveModulePositions[1] = swerveDrive.getModulePositions()[1];
                swerveDriveModulePositions[2] = swerveDrive.getModulePositions()[2];
                swerveDriveModulePositions[3] = swerveDrive.getModulePositions()[3];

                updateOdometryPose(swerveDriveModulePositions);
            }
        }

        if (dashboardCounter++ >= 5) {
            if (Constants.debugDashboard) {
                SmartDashboard.putNumber("Odometry Rotation (deg)", latestSwervePose.getRotation().getDegrees());
                SmartDashboard.putNumber("Odometry X (in)", (latestSwervePose.getX() * (100 / 2.54)));
                SmartDashboard.putNumber("Odometry Y (in)", (latestSwervePose.getY() * (100 / 2.54)));
                SmartDashboard.putNumber("Odometry X (m)", latestSwervePose.getX());
                SmartDashboard.putNumber("Odometry Y (m)", latestSwervePose.getY());
                SmartDashboard.putBoolean("Is reading Odom", odomReadingTesting);

                // SmartDashboard.putNumber("Front Left Wheel Pos", frontLeftModule.getWheelPositionMeters());
                // SmartDashboard.putNumber("Front Right Wheel Pos", frontRightModule.getWheelPositionMeters());
                // SmartDashboard.putNumber("Back Left Wheel Pos", rearLeftModule.getWheelPositionMeters());
                // SmartDashboard.putNumber("Back Right Wheel Pos", rearRightModule.getWheelPositionMeters());

                SmartDashboard.putNumber("Front Left Wheel Pos", swerveDrive.getModulePositions()[0].distanceMeters);
                SmartDashboard.putNumber("Front Right Wheel Pos", swerveDrive.getModulePositions()[1].distanceMeters);
                SmartDashboard.putNumber("Back Left Wheel Pos", swerveDrive.getModulePositions()[2].distanceMeters);
                SmartDashboard.putNumber("Back Right Wheel Pos", swerveDrive.getModulePositions()[3].distanceMeters);

                // SmartDashboard.putNumber("Front Left Encoder Angle", frontLeftModule.getEncoderAngle());
                // SmartDashboard.putNumber("Front Right Encoder Angle", frontRightModule.getEncoderAngle());
                // SmartDashboard.putNumber("Back Left Encoder Angle", rearLeftModule.getEncoderAngle());
                // SmartDashboard.putNumber("Back Right Encoder Angle", rearRightModule.getEncoderAngle());

                SmartDashboard.putNumber("Front Left Encoder Angle", swerveDrive.getModulePositions()[0].angle.getDegrees());
                SmartDashboard.putNumber("Front Right Encoder Angle", swerveDrive.getModulePositions()[1].angle.getDegrees());
                SmartDashboard.putNumber("Back Left Encoder Angle", swerveDrive.getModulePositions()[2].angle.getDegrees());
                SmartDashboard.putNumber("Back Right Encoder Angle", swerveDrive.getModulePositions()[3].angle.getDegrees());

                // SmartDashboard.putNumber("Gyro Yaw (raw deg)", navx.getYaw());
                SmartDashboard.putNumber("Gyro Yaw (raw deg)", swerveDrive.getYaw().getDegrees());
                SmartDashboard.putNumber("Gyro Yaw (adj deg)", getGyroYaw());

                SmartDashboard.putBoolean("IsAutoRotate", isAutoRotate());

                SmartDashboard.putData("Drivetrain", this);
            }

            if (Constants.debugDashboard) {
                // SmartDashboard.putNumber("Wheel speed (m)", frontLeftModule.getWheelSpeedMeters());
                SmartDashboard.putNumber("Wheel speed (m)", swerveDrive.getRobotVelocity().vxMetersPerSecond);
                SmartDashboard.putBoolean(
                    "LL Seeing Target?",
                    (limeLightCameraBack.getTargetID() > -1) || (limeLightCameraFront.getTargetID() > -1) ||
                        (limeLightCameraLeft.getTargetID() > -1) || (limeLightCameraRight.getTargetID() > -1));
                SmartDashboard.putBoolean("Latest LL Pose OK?", latestVisionPoseValid);

                if (limelightFrontBotPose != null && limelightFrontBotPose.length >= 6) {
                    SmartDashboard.putNumber("LL Front X (m)",
                            limelightFrontBotPose[0]);
                    SmartDashboard.putNumber("LL Front Y (m)",
                            limelightFrontBotPose[1]);
                    SmartDashboard.putNumber("LL Front Yaw (deg)",
                            limelightFrontBotPose[5]);
                    // SmartDashboard.putNumber("LL Front Latency (ms)",
                    // limelightFrontBotPose[6]);
                    SmartDashboard.putNumber("LL Front Tid",
                            limeLightCameraFront.getTargetID());
                    SmartDashboard.putNumber("LL Front Ta",
                            limeLightCameraFront.getTargetArea());
                }
                if (limelightBackBotPose != null && limelightBackBotPose.length >= 6) {
                    SmartDashboard.putNumber("LL Back X (m)",
                            limelightBackBotPose[0]);
                    SmartDashboard.putNumber("LL Back Y (m)",
                            limelightBackBotPose[1]);
                    SmartDashboard.putNumber("LL Back Yaw (deg)",
                            limelightBackBotPose[5]);
                    // SmartDashboard.putNumber("LL Back Latency (ms)",
                    // limelightBackBotPose[6]);
                    SmartDashboard.putNumber("LL Back Tid",
                            limeLightCameraBack.getTargetID());
                    SmartDashboard.putNumber("LL Back Ta",
                            limeLightCameraBack.getTargetArea());
                }
                if (limelightLeftBotPose != null && limelightLeftBotPose.length >= 6) {
                    SmartDashboard.putNumber("LL Left X (m)",
                            limelightLeftBotPose[0]);
                    SmartDashboard.putNumber("LL Left Y (m)",
                            limelightLeftBotPose[1]);
                    SmartDashboard.putNumber("LL Left Yaw (deg)",
                            limelightLeftBotPose[5]);
                    // SmartDashboard.putNumber("LL Left Latency (ms)",
                    // limelightLeftBotPose[6]);
                    SmartDashboard.putNumber("LL Left Tid",
                            limeLightCameraLeft.getTargetID());
                    SmartDashboard.putNumber("LL Left Ta",
                            limeLightCameraLeft.getTargetArea());
                }
                if (limelightRightBotPose != null && limelightRightBotPose.length >= 6) {
                    SmartDashboard.putNumber("LL Right X (m)",
                            limelightRightBotPose[0]);
                    SmartDashboard.putNumber("LL Right Y (m)",
                            limelightRightBotPose[1]);
                    SmartDashboard.putNumber("LL Right Yaw (deg)",
                            limelightRightBotPose[5]);
                    // SmartDashboard.putNumber("LL Right Latency (ms)",
                    // limelightRightBotPose[6]);
                    SmartDashboard.putNumber("LL Right Tid",
                            limeLightCameraRight.getTargetID());
                    SmartDashboard.putNumber("LL Right Ta",
                            limeLightCameraRight.getTargetArea());
                }

                if (latestVisionPose != null) {
                    SmartDashboard.putNumber("LL All X (m)", latestVisionPose.getX());
                    SmartDashboard.putNumber("LL All Y (m)", latestVisionPose.getY());
                    SmartDashboard.putNumber("LL All Th (deg)", latestVisionPose.getRotation().getDegrees());
                    SmartDashboard.putNumber("LL All FPGATime", Timer.getFPGATimestamp());
                    SmartDashboard.putNumber("LL All Latency", limeLightBlendedLatency);
                }

                SmartDashboard.putBoolean("LL updating Odom", useLimelightOdometryUpdates);
                SmartDashboard.putBoolean("Latest LL Pose Valid?", latestVisionPoseValid);
                SmartDashboard.putNumber("LL Total Area", limelightTotalArea);

            }
            dashboardCounter = 0;
        }
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

public void resetOdometry() {
    if (this.simpleOdometryReset) {
        swerveDrivePoseEstimator.resetPosition(
                Rotation2d.fromDegrees(-getGyroYaw()), swerveDriveModulePositions,
                new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(-getGyroYaw())));
    } else {
        // double newGyroOffset = navx.getYaw() - resetPose.getRotation().getDegrees();
        double newGyroOffset = swerveDrive.getYaw().getDegrees() - resetPose.getRotation().getDegrees();
        if (getAllianceCoordinateSpace() == CoordinateSpace.Blue) {
            // Blue so just set gyro offset normally
            setGyroOffset(newGyroOffset);
        } else {
            // Red so we have to flip gyro by 180
            newGyroOffset += 180.0;
            if (newGyroOffset > 180) {
                newGyroOffset -= 360.0;
            }
            setGyroOffset(newGyroOffset);
        }
        swerveDrivePoseEstimator.resetPosition(
                Rotation2d.fromDegrees(-getGyroYaw()),
                swerveDriveModulePositions,
                resetPose);
        latestSwervePose = resetPose;
    }
}

public boolean useLimeLightForOdometry() {
    return useLimelightOdometryUpdates;
}

public void setLimeLightOdometryUpdates(boolean isUpdating) {
    useLimelightOdometryUpdates = isUpdating;
}

public void calculateBlendedVisionPose() {
        double x = 0.0;
        double y = 0.0;
        double theta = 0.0;
        limelightTotalArea = 0.0;
        limeLightBlendedLatency = 0.0;
        latestVisionPoseValid = false;

        SmartDashboard.putString("Alliance Color", getAllianceCoordinateSpace().toString());
        limelightFrontBotPose = limeLightCameraFront.getBotPose(CoordinateSpace.Blue);
        limelightBackBotPose = limeLightCameraBack.getBotPose(CoordinateSpace.Blue);
        limelightLeftBotPose = limeLightCameraLeft.getBotPose(CoordinateSpace.Blue);
        limelightRightBotPose = limeLightCameraRight.getBotPose(CoordinateSpace.Blue);

        if (limelightFrontBotPose != null && limeLightCameraFront.getTargetID() != -1) {
            double limelightFrontTargetArea = limeLightCameraFront.getTargetArea();
            if (limelightFrontTargetArea >= Constants.Vision.targetAreaThresholdLL3) {
                limelightTotalArea += limelightFrontTargetArea;
                x += limelightFrontBotPose[0] * limelightFrontTargetArea;
                y += limelightFrontBotPose[1] * limelightFrontTargetArea;
                theta += limelightFrontBotPose[5] * limelightFrontTargetArea;
                limeLightBlendedLatency += limelightFrontBotPose[6];
            }
        }
        if (limelightBackBotPose != null && limeLightCameraBack.getTargetID() != -1) {
            double limelightBackTargetArea = limeLightCameraBack.getTargetArea();
            if (limelightBackTargetArea >= Constants.Vision.targetAreaThresholdLL3) {
                limelightTotalArea += limelightBackTargetArea;
                x += limelightBackBotPose[0] * limelightBackTargetArea;
                y += limelightBackBotPose[1] * limelightBackTargetArea;
                theta += limelightBackBotPose[5] * limelightBackTargetArea;
                limeLightBlendedLatency += limelightBackBotPose[6];
            }
        }
        if (limelightLeftBotPose != null && limeLightCameraLeft.getTargetID() != -1) {
            double limelightLeftTargetArea = limeLightCameraLeft.getTargetArea();
            if (limelightLeftTargetArea >= Constants.Vision.targetAreaThresholdLL2) {
                limelightTotalArea += limelightLeftTargetArea;
                x += limelightLeftBotPose[0] * limelightLeftTargetArea;
                y += limelightLeftBotPose[1] * limelightLeftTargetArea;
                theta += limelightLeftBotPose[5] * limelightLeftTargetArea;
                limeLightBlendedLatency += limelightLeftBotPose[6];
            }
        }
        if (limelightRightBotPose != null && limeLightCameraRight.getTargetID() != -1) {
            double limelightRightTargetArea = limeLightCameraRight.getTargetArea();
            if (limelightRightTargetArea >= Constants.Vision.targetAreaThresholdLL2) {
                limelightTotalArea += limelightRightTargetArea;
                x += limelightRightBotPose[0] * limelightRightTargetArea;
                y += limelightRightBotPose[1] * limelightRightTargetArea;
                theta += limelightRightBotPose[5] * limelightRightTargetArea;
                limeLightBlendedLatency += limelightRightBotPose[6];
            }
        }

        if (limelightTotalArea <= Constants.Vision.totalTargetAreaThreshold) {
            limeLightBlendedLatency = 0.0;
            latestVisionPoseValid = false;
            isUpdatingLimelightOdometry = false;
            limelightXMedianFilter.reset();
            limelightYMedianFilter.reset();
            limelightAngleMedianFilter.reset();
            return;
        } else {
            latestVisionPoseValid = true;
            isUpdatingLimelightOdometry = true;

            x /= limelightTotalArea;
            y /= limelightTotalArea;
            theta /= limelightTotalArea;
 
            limeLightBlendedLatency /= limelightTotalArea;

            latestVisionPose = new Pose2d(
                    limelightXMedianFilter.calculate(x),
                    limelightYMedianFilter.calculate(y),
                    Rotation2d.fromDegrees(limelightAngleMedianFilter.calculate(theta)));
            isUpdatingLimelightOdometry = false;
            return;
        }
    }

    public void updateOdometryPose(SwerveModulePosition[] modulePositions) {
        odomReadingTesting = true;
        this.latestSwervePose = this.swerveDrivePoseEstimator.updateWithTime(
                Timer.getFPGATimestamp(),
                Rotation2d.fromDegrees(-getGyroYaw()),
                modulePositions);
    }

    public boolean isInSlowMode() {
        return inSlowMode;
    }

    public void setInSlowMode(boolean inSlowMode) {
        this.inSlowMode = inSlowMode;
    }

    public boolean getDoFieldOrient() {
        return doFieldOrient;
    }

    public void setDoFieldOrient(boolean disableFieldOrient) {
        this.doFieldOrient = disableFieldOrient;
    }

    public boolean isAutoRotate() {
        return this.autoRotate;
    }

    public void setAutoRotate(boolean autoRotateMode) {
        this.autoRotate = autoRotateMode;
    }

    public boolean isLoadingMode() {
        return loadingMode;
    }

    public void setLoadingMode(boolean loadingMode) {
        this.loadingMode = loadingMode;
    }

    public void resetSubsystemState() {
        inSlowMode = true;
        doFieldOrient = Constants.DrivetrainConstants.isFieldCentric;
        autoRotate = false;
        loadingMode = false;
        limelightXMedianFilter.reset();
        limelightYMedianFilter.reset();
        limelightAngleMedianFilter.reset();
   }

   // Move robot straight at a heading and speed
   public void moveRobotFrontBack(boolean forward, double velocity) {

    // Calculate the Swerve States
    ChassisSpeeds swerveStates;

    double y1 = velocity / Constants.DrivetrainConstants.swerveMaxSpeed;
    if (!forward) {
        y1 *= -1;
    }

    // swerveStates = swerveController.calculate(0.0, y1, 0.0);
    swerveStates = swerveDrive.swerveController.getTargetSpeeds(0.0, y1, 0.0, swerveDrive.getOdometryHeading().getRadians(), maxSpeed);

    // Command the Swerve Modules
    // frontLeft.setDriveVelocity(swerveStates[0], swerveStates[1]);
    // frontRight.setDriveVelocity(swerveStates[2], swerveStates[3]);
    // rearLeft.setDriveVelocity(swerveStates[4], swerveStates[5]);
    // rearRight.setDriveVelocity(swerveStates[6], swerveStates[7]);
    swerveDrive.setChassisSpeeds(swerveStates);
    // frontLeft.setDrive(velocity, 0);
    // frontRight.setDrive(velocity, 0);
    // rearLeft.setDrive(velocity, 0);
    // rearRight.setDrive(velocity, 0);
}

public void turnRobot( double heading) {

    // Calculate the Swerve States
    ChassisSpeeds swerveStates;

    double x2 = getGyroYaw() - heading;

    if (x2 > 180) {
         x2 -= 360;
    } else if (x2 < -180) {
        x2 += 360;
    }
    if (Math.abs(x2 - heading) > Constants.DrivetrainConstants.autoAngleThreshold) {
        x2 = x2 * Constants.DrivetrainConstants.driveRotationP;
    } else {
        x2 = 0.0;
    }

x2 = Math.min(x2, .90);
x2 = Math.max(x2, -.90);


    // swerveStates = swerveController.calculate(0.0, 0.0, x2);
    swerveStates = swerveDrive.swerveController.getTargetSpeeds(0.0, 0.0, x2, swerveDrive.getOdometryHeading().getRadians(), maxSpeed);

    // Command the Swerve Modules
    // frontLeft.setDriveVelocity(swerveStates[0], swerveStates[1]);
    // frontRight.setDriveVelocity(swerveStates[2], swerveStates[3]);
    // rearLeft.setDriveVelocity(swerveStates[4], swerveStates[5]);
    // rearRight.setDriveVelocity(swerveStates[6], swerveStates[7]);
    swerveDrive.setChassisSpeeds(swerveStates);
}
}
