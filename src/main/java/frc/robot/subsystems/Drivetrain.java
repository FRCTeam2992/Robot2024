// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
// import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

/**
 * Use a bugfix posted on ChiefDelphi by Programming4907 instead of the origin dependency
 * https://www.chiefdelphi.com/t/navx2-disconnecting-reconnecting-intermittently-not-browning-out/425487/42
 * The src/lib/NavX folder was copied wholesale from Thunderstamps/navx2workaround
 * and package references in each file were adapted to the location in this repo.
 */
// import com.kauailabs.navx.frc.AHRS;

import frc.lib.NavX.AHRS;

import frc.lib.drive.swerve.SwerveController;
import frc.lib.drive.swerve.SwerveModuleFalconFalcon;
import frc.lib.vision.LimeLight;
import frc.lib.vision.LimelightHelpers;
import frc.lib.vision.LimeLight.CoordinateSpace;
import frc.lib.vision.LimeLight.LimeLightModel;
import frc.lib.vision.LimeLight.StreamMode;
import frc.lib.vision.LimelightHelpers.PoseEstimate;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleArrayLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Drivetrain extends SubsystemBase {
    /** Creates a new Drivetrain. */
    // Motor Contollers
    private TalonFX frontLeftDrive;
    private TalonFX frontLeftTurn;

    private TalonFX frontRightDrive;
    private TalonFX frontRightTurn;

    private TalonFX rearLeftDrive;
    private TalonFX rearLeftTurn;

    private TalonFX rearRightDrive;
    private TalonFX rearRightTurn;

    private TalonFXConfiguration driveMotorConfigs;
    private TalonFXConfiguration turnMotorConfigs;

    // Control Requests
    private DutyCycleOut percentControlRequest;
    private VelocityDutyCycle velocityControlRequest;

    // Module CAN Encoders
    private final CANcoder frontLeftEncoder;
    private final CANcoder frontRightEncoder;
    private final CANcoder rearLeftEncoder;
    private final CANcoder rearRightEncoder;

    private CANcoderConfiguration encoderConfigs;

    // Turn PID Controllers
    private final PIDController frontLeftController;
    private final PIDController frontRightController;
    private final PIDController rearLeftController;
    private final PIDController rearRightController;

    // Swerve Modules
    public final SwerveModuleFalconFalcon frontLeftModule;
    public final SwerveModuleFalconFalcon frontRightModule;
    public final SwerveModuleFalconFalcon rearLeftModule;
    public final SwerveModuleFalconFalcon rearRightModule;

    // Swerve Controller
    public final SwerveController swerveController;

    // Limelights
    public final LimeLight limeLightCameraFront;
    public final LimeLight limeLightCameraBack;
    public final LimeLight limeLightCameraLeft;
    public final LimeLight limeLightCameraRight;
    public final ArrayList<LimeLight> limelightList;
    public double limeLightBlendedLatency = 0.0;
    private MedianFilter limelightXMedianFilter;
    private MedianFilter limelightYMedianFilter;
    private MedianFilter limelightAngleMedianFilter;
    private Translation2d lastOdometryTranslation = new Translation2d();
    private double lastDistanceMoved = 0.0;

    private DataLog mDataLog;
    private LimelightHelpers.PoseEstimate tempPoseEstimate;
    private final Field2d fieldForPosePublishing = new Field2d();
    private final Field2d fieldForFrontPosePublishing = new Field2d();
    private final Field2d fieldForBackPosePublishing = new Field2d();
    private final Field2d fieldForLeftPosePublishing = new Field2d();
    private final Field2d fieldForRightPosePublishing = new Field2d();

    private DoubleArrayLogEntry swerveCANCoderLog;

    // Robot Gyro
    private AHRS navx;
    private double gyroOffset = 0.0;

    public Pose2d latestSwervePose = new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0.0));
    public Pose2d latestVisionPose = new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0.0));
    public boolean latestVisionPoseValid = false; // Do we have a current vision sighting?


    // Swerve Drive Kinematics
    public final SwerveDriveKinematics swerveDriveKinematics;

    // Swerve Drive Odometry
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
    private int odometryResetCount = 0;
    private int lastOdometryResetCount = -1;
    private boolean simpleOdometryReset = true;
    public Pose2d resetPose = Constants.DrivetrainConstants.zeroPose;
    private double endgameTargetAngle = 180.0;

    private boolean odomReadingTesting = false;

    private int dashboardCounter = 0;

    public Drivetrain() {

        driveMotorConfigs = new TalonFXConfiguration();
        turnMotorConfigs = new TalonFXConfiguration();

        encoderConfigs = new CANcoderConfiguration();

        // Controll Requests
        percentControlRequest = new DutyCycleOut(0.0);
        velocityControlRequest = new VelocityDutyCycle(0.0);

        // Motor Inits
        frontLeftDrive = new TalonFX(Constants.DrivetrainConstants.CanIDs.frontLeftDrive, "CanBus2");
        initTalonFX(frontLeftDrive, driveMotorConfigs, InvertedValue.Clockwise_Positive);

        frontLeftTurn = new TalonFX(Constants.DrivetrainConstants.CanIDs.frontLeftTurn, "CanBus2");
        initTalonFX(frontLeftTurn, turnMotorConfigs, InvertedValue.CounterClockwise_Positive);

        frontRightDrive = new TalonFX(Constants.DrivetrainConstants.CanIDs.frontRightDrive, "CanBus2");
        initTalonFX(frontRightDrive, driveMotorConfigs, InvertedValue.Clockwise_Positive);

        frontRightTurn = new TalonFX(Constants.DrivetrainConstants.CanIDs.frontRightTurn, "CanBus2");
        initTalonFX(frontRightTurn, turnMotorConfigs, InvertedValue.CounterClockwise_Positive);

        rearRightDrive = new TalonFX(Constants.DrivetrainConstants.CanIDs.rearRightDrive, "CanBus2");
        initTalonFX(rearRightDrive, driveMotorConfigs, InvertedValue.Clockwise_Positive);

        rearRightTurn = new TalonFX(Constants.DrivetrainConstants.CanIDs.rearRightTurn, "CanBus2");
        initTalonFX(rearRightTurn, turnMotorConfigs, InvertedValue.CounterClockwise_Positive);

        rearLeftDrive = new TalonFX(Constants.DrivetrainConstants.CanIDs.rearLeftDrive, "CanBus2");
        initTalonFX(rearLeftDrive, driveMotorConfigs, InvertedValue.Clockwise_Positive);

        rearLeftTurn = new TalonFX(Constants.DrivetrainConstants.CanIDs.rearLeftTurn, "CanBus2");
        initTalonFX(rearLeftTurn, turnMotorConfigs, InvertedValue.CounterClockwise_Positive);

        frontRightEncoder = new CANcoder(Constants.DrivetrainConstants.CanIDs.frontRightEncoder, "CanBus2");
        initCANCoder(frontRightEncoder, AbsoluteSensorRangeValue.Signed_PlusMinusHalf,
                SensorDirectionValue.Clockwise_Positive);

        frontLeftEncoder = new CANcoder(Constants.DrivetrainConstants.CanIDs.frontLeftEncoder, "CanBus2");
        initCANCoder(frontLeftEncoder, AbsoluteSensorRangeValue.Signed_PlusMinusHalf,
                SensorDirectionValue.Clockwise_Positive);

        rearRightEncoder = new CANcoder(Constants.DrivetrainConstants.CanIDs.rearRightEncoder, "CanBus2");
        initCANCoder(rearRightEncoder, AbsoluteSensorRangeValue.Signed_PlusMinusHalf,
                SensorDirectionValue.Clockwise_Positive);

        rearLeftEncoder = new CANcoder(Constants.DrivetrainConstants.CanIDs.rearLeftEncoder, "CanBus2");
        initCANCoder(rearLeftEncoder, AbsoluteSensorRangeValue.Signed_PlusMinusHalf,
                SensorDirectionValue.Clockwise_Positive);

        setDriveNeutralMode(NeutralModeValue.Coast);
        setTurnNeutralMode(NeutralModeValue.Brake);

        setDriveCurrentLimit(40.0, 50.0);
        setTurnCurrentLimit(60.0); // potentially unused

        frontLeftController = new PIDController(Constants.DrivetrainConstants.PIDConstants.turnP,
                Constants.DrivetrainConstants.PIDConstants.turnI,
                Constants.DrivetrainConstants.PIDConstants.turnD);
        frontLeftController.enableContinuousInput(-180.0, 180.0);
        frontLeftController.setTolerance(2.0);

        frontRightController = new PIDController(Constants.DrivetrainConstants.PIDConstants.turnP,
                Constants.DrivetrainConstants.PIDConstants.turnI,
                Constants.DrivetrainConstants.PIDConstants.turnD);
        frontRightController.enableContinuousInput(-180.0, 180.0);
        frontRightController.setTolerance(2.0);

        rearLeftController = new PIDController(Constants.DrivetrainConstants.PIDConstants.turnP,
                Constants.DrivetrainConstants.PIDConstants.turnI,
                Constants.DrivetrainConstants.PIDConstants.turnD);
        rearLeftController.enableContinuousInput(-180.0, 180.0);
        rearLeftController.setTolerance(2.0);

        rearRightController = new PIDController(Constants.DrivetrainConstants.PIDConstants.turnP,
                Constants.DrivetrainConstants.PIDConstants.turnI,
                Constants.DrivetrainConstants.PIDConstants.turnD);
        rearRightController.enableContinuousInput(-180.0, 180.0);
        rearRightController.setTolerance(2.0);

        // Set the Drive PID Controllers
        driveMotorConfigs.Slot0.kP = Constants.DrivetrainConstants.PIDConstants.driveP;
        driveMotorConfigs.Slot0.kI = Constants.DrivetrainConstants.PIDConstants.driveI;
        driveMotorConfigs.Slot0.kD = Constants.DrivetrainConstants.PIDConstants.driveD;
        driveMotorConfigs.Slot0.kV = Constants.DrivetrainConstants.PIDConstants.driveV;

        frontLeftDrive.getConfigurator().apply(driveMotorConfigs);
        frontRightDrive.getConfigurator().apply(driveMotorConfigs);
        rearLeftDrive.getConfigurator().apply(driveMotorConfigs);
        rearRightDrive.getConfigurator().apply(driveMotorConfigs);

        // Swerve Modules

        frontLeftModule = new SwerveModuleFalconFalcon(frontLeftDrive, frontLeftTurn, frontLeftEncoder,
                Constants.DrivetrainConstants.frontLeftOffset, frontLeftController,
                Constants.DrivetrainConstants.driveWheelDiameter,
                Constants.DrivetrainConstants.driveGearRatio,
                Constants.DrivetrainConstants.swerveMaxSpeed, percentControlRequest, velocityControlRequest);

        frontRightModule = new SwerveModuleFalconFalcon(frontRightDrive, frontRightTurn, frontRightEncoder,
                Constants.DrivetrainConstants.frontRightOffset, frontRightController,
                Constants.DrivetrainConstants.driveWheelDiameter,
                Constants.DrivetrainConstants.driveGearRatio,
                Constants.DrivetrainConstants.swerveMaxSpeed, percentControlRequest, velocityControlRequest);

        rearLeftModule = new SwerveModuleFalconFalcon(rearLeftDrive, rearLeftTurn, rearLeftEncoder,
                Constants.DrivetrainConstants.rearLeftOffset,
                rearLeftController, Constants.DrivetrainConstants.driveWheelDiameter,
                Constants.DrivetrainConstants.driveGearRatio,
                Constants.DrivetrainConstants.swerveMaxSpeed, percentControlRequest, velocityControlRequest);

        rearRightModule = new SwerveModuleFalconFalcon(rearRightDrive, rearRightTurn, rearRightEncoder,
                Constants.DrivetrainConstants.rearRightOffset, rearRightController,
                Constants.DrivetrainConstants.driveWheelDiameter,
                Constants.DrivetrainConstants.driveGearRatio,
                Constants.DrivetrainConstants.swerveMaxSpeed, percentControlRequest, velocityControlRequest);

        // Swerve Controller
        swerveController = new SwerveController(
                Constants.DrivetrainConstants.swerveLength,
                Constants.DrivetrainConstants.swerveWidth);

        // Limelight
        limeLightCameraFront = new LimeLight("limelight-front", LimeLightModel.LL3);
        limeLightCameraBack = new LimeLight("limelight-back", LimeLightModel.LL3);
        limeLightCameraLeft = new LimeLight("limelight-left", LimeLightModel.LL2);
        limeLightCameraLeft.setStreamMode(StreamMode.PiPSecondary);
        limeLightCameraRight = new LimeLight("limelight-right", LimeLightModel.LL2);

        limelightList = new ArrayList<LimeLight>();
        limelightList.add(limeLightCameraFront);
        limelightList.add(limeLightCameraBack);
        limelightList.add(limeLightCameraLeft);
        limelightList.add(limeLightCameraRight);

        limelightXMedianFilter = new MedianFilter(1);
        limelightYMedianFilter = new MedianFilter(1);
        limelightAngleMedianFilter = new MedianFilter(1);

        if (Constants.dataLogging) {
            mDataLog = DataLogManager.getLog();
            swerveCANCoderLog = new DoubleArrayLogEntry(mDataLog, "/Drivetrain/Swerve/CANCoders");
        }

        // robot gyro initialization
        navx = new AHRS(SPI.Port.kMXP, Constants.DrivetrainConstants.gyroUpdateRateHz);

        swerveDriveModulePositions[0] = frontLeftModule.getPosition();
        swerveDriveModulePositions[1] = frontRightModule.getPosition();
        swerveDriveModulePositions[2] = rearLeftModule.getPosition();
        swerveDriveModulePositions[3] = rearRightModule.getPosition();

        // Swerve Drive Kinematics
        swerveDriveKinematics = new SwerveDriveKinematics(
                Constants.DrivetrainConstants.frontLeftLocation,
                Constants.DrivetrainConstants.frontRightLocation,
                Constants.DrivetrainConstants.rearLeftLocation,
                Constants.DrivetrainConstants.rearRightLocation);

        // Serve Drive Odometry
        swerveDrivePoseEstimator = new SwerveDrivePoseEstimator(
                swerveDriveKinematics,
                Rotation2d.fromDegrees(-getGyroYaw()),
                swerveDriveModulePositions,
                new Pose2d(0.0, 0.0, new Rotation2d()),
                // State measurement standard deviations. X, Y, theta.
                VecBuilder.fill(0.1, 0.1, 0.1),
                // Global measurement standard deviations. X, Y, and theta.
                VecBuilder.fill(0.9, 0.9, 9999999.0));

        // Configure AutoBuilder last
        AutoBuilder.configureHolonomic(
                this::getLatestSwervePose, // Robot pose supplier
                this::scheduleOdometryReset, // Method to reset odometry (will be called if your auto has a starting
                                             // pose)
                this::getRobotChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                this::driveRobotFromChassisSpeeds, // Method that will drive the robot given ROBOT RELATIVE
                                                   // ChassisSpeeds
                new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your
                                                 // Constants class
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
                        Constants.DrivetrainConstants.maxPathFollowingVelocity, // Max module speed, in m/s
                        Constants.DrivetrainConstants.driveBaseRadius, // Drive base radius in meters. Distance from
                                                                       // robot center to furthest module.
                        new ReplanningConfig() // Default path replanning config. See the API for the options here
                ),
                () -> {
                    // Boolean supplier that controls when the path will be mirrored for the red
                    // alliance
                    // This will flip the path being followed to the red side of the field.
                    // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                },
                this // Reference to this subsystem to set requirements
        );

    }

    private void initTalonFX(TalonFX motorContollerName, TalonFXConfiguration configs, InvertedValue motorDirection) {
        configs.MotorOutput.Inverted = motorDirection;
        motorContollerName.getConfigurator().apply(configs);
    }

    private void initCANCoder(CANcoder CANCoderName, AbsoluteSensorRangeValue sensorRange,
            SensorDirectionValue sensorDirection) {
        encoderConfigs.MagnetSensor.AbsoluteSensorRange = sensorRange;
        encoderConfigs.MagnetSensor.SensorDirection = sensorDirection;
        CANCoderName.getConfigurator().apply(encoderConfigs);
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
                for (LimeLight limelight: limelightList) {
                    limelight.setRobotOrientation(-getGyroYaw());
                    if (Constants.debugDashboard) {
                        tempPoseEstimate = limelight.getLimelightMeasurement();
                        if (tempPoseEstimate != null) {
                            if (limelight.networkTableName == limeLightCameraBack.networkTableName) {
                                fieldForBackPosePublishing.setRobotPose(tempPoseEstimate.pose);
                                SmartDashboard.putData("LL Field " + limelight.networkTableName, fieldForBackPosePublishing);
                            } else if (limelight.networkTableName == limeLightCameraFront.networkTableName) {
                                fieldForFrontPosePublishing.setRobotPose(tempPoseEstimate.pose);
                                SmartDashboard.putData("LL Field " + limelight.networkTableName, fieldForFrontPosePublishing);
                            } else if (limelight.networkTableName == limeLightCameraLeft.networkTableName) {
                                fieldForLeftPosePublishing.setRobotPose(tempPoseEstimate.pose);
                                SmartDashboard.putData("LL Field " + limelight.networkTableName, fieldForLeftPosePublishing);
                            } else if (limelight.networkTableName == limeLightCameraRight.networkTableName) {
                                fieldForRightPosePublishing.setRobotPose(tempPoseEstimate.pose);
                                SmartDashboard.putData("LL Field " + limelight.networkTableName, fieldForRightPosePublishing);
                            }
                            SmartDashboard.putNumber("LL #Tags " + limelight.networkTableName,
                                    tempPoseEstimate.tagCount);
                            SmartDashboard.putNumber("LL Dist " + limelight.networkTableName, tempPoseEstimate.avgTagDist);
                            SmartDashboard.putNumber("LL Area " + limelight.networkTableName, tempPoseEstimate.avgTagArea);
                            SmartDashboard.putNumber("LL Latency " + limelight.networkTableName, tempPoseEstimate.latency);
                        } else {
                            SmartDashboard.putNumber("LL #Tags " + limelight.networkTableName, 0.0);
                            SmartDashboard.putNumber("LL Dist " + limelight.networkTableName, -1.0);
                            SmartDashboard.putNumber("LL Area " + limelight.networkTableName, -1.0);
                            SmartDashboard.putNumber("LL Latency " + limelight.networkTableName, -1.0);
                        }
                    }
                }

                if (useLimeLightForOdometry()) {
                    doLimelightPoseUpdate();
                }

                // Update odometry via wheel encoders
                swerveDriveModulePositions[0] = frontLeftModule.getPosition();
                swerveDriveModulePositions[1] = frontRightModule.getPosition();
                swerveDriveModulePositions[2] = rearLeftModule.getPosition();
                swerveDriveModulePositions[3] = rearRightModule.getPosition();

                updateOdometryPose(swerveDriveModulePositions);
            }
        }

        if (Constants.dataLogging && DriverStation.isEnabled()) {

            double[] canReadings = { frontLeftModule.getEncoderAngle(), frontRightModule.getEncoderAngle(),
                    rearLeftModule.getEncoderAngle(), rearRightModule.getEncoderAngle() };
            swerveCANCoderLog.append(canReadings);
        }

        if (dashboardCounter++ >= 5) {
            if (Constants.debugDashboard) {
                SmartDashboard.putData("Drivetrain", this);

                SmartDashboard.putNumber("Odometry Rotation (deg)", latestSwervePose.getRotation().getDegrees());
                SmartDashboard.putNumber("Odometry X (in)", (latestSwervePose.getX() * (100 / 2.54)));
                SmartDashboard.putNumber("Odometry Y (in)", (latestSwervePose.getY() * (100 / 2.54)));
                SmartDashboard.putNumber("Odometry X (m)", latestSwervePose.getX());
                SmartDashboard.putNumber("Odometry Y (m)", latestSwervePose.getY());
                SmartDashboard.putBoolean("Is reading Odom", odomReadingTesting);

                SmartDashboard.putNumber("Front Left Wheel Pos", frontLeftModule.getWheelPositionMeters());
                SmartDashboard.putNumber("Front Right Wheel Pos", frontRightModule.getWheelPositionMeters());
                SmartDashboard.putNumber("Back Left Wheel Pos", rearLeftModule.getWheelPositionMeters());
                SmartDashboard.putNumber("Back Right Wheel Pos", rearRightModule.getWheelPositionMeters());

                SmartDashboard.putNumber("Front Left Encoder Angle", frontLeftModule.getEncoderAngle());
                SmartDashboard.putNumber("Front Right Encoder Angle", frontRightModule.getEncoderAngle());
                SmartDashboard.putNumber("Back Left Encoder Angle", rearLeftModule.getEncoderAngle());
                SmartDashboard.putNumber("Back Right Encoder Angle", rearRightModule.getEncoderAngle());

                SmartDashboard.putNumber("Gyro Yaw (raw deg)", navx.getYaw());
                SmartDashboard.putNumber("Gyro Yaw (adj deg)", getGyroYaw());

                SmartDashboard.putBoolean("IsAutoRotate", isAutoRotate());
                SmartDashboard.putNumber("Speaker Aim Difference", calculateDistanceFromSpeaker());

                SmartDashboard.putNumber("FL wheel speed (m)", frontLeftModule.getWheelSpeedMeters());
                SmartDashboard.putNumber("FR wheel speed (m)", frontRightModule.getWheelSpeedMeters());
                SmartDashboard.putNumber("RL wheel speed (m)", rearLeftModule.getWheelSpeedMeters());
                SmartDashboard.putNumber("RR wheel speed (m)", rearRightModule.getWheelSpeedMeters());
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

    public void setDriveNeutralMode(NeutralModeValue mode) {
        driveMotorConfigs.MotorOutput.NeutralMode = mode;

        frontLeftDrive.getConfigurator().apply(driveMotorConfigs);
        frontRightDrive.getConfigurator().apply(driveMotorConfigs);
        rearLeftDrive.getConfigurator().apply(driveMotorConfigs);
        rearRightDrive.getConfigurator().apply(driveMotorConfigs);

    }

    public void setTurnNeutralMode(NeutralModeValue mode) {
        turnMotorConfigs.MotorOutput.NeutralMode = mode;

        frontLeftTurn.getConfigurator().apply(turnMotorConfigs);
        frontRightTurn.getConfigurator().apply(turnMotorConfigs);
        rearLeftTurn.getConfigurator().apply(turnMotorConfigs);
        rearRightTurn.getConfigurator().apply(turnMotorConfigs);
    }

    public void setDriveCurrentLimit(double currentLimit, double triggerCurrent) {

        driveMotorConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
        driveMotorConfigs.CurrentLimits.StatorCurrentLimit = currentLimit;
        // driveMotorConfigs.CurrentLimits.SupplyCurrentThreshold = triggerCurrent;
        // driveMotorConfigs.CurrentLimits.SupplyTimeThreshold = 0.02;

        frontLeftDrive.getConfigurator().apply(driveMotorConfigs);
        frontRightDrive.getConfigurator().apply(driveMotorConfigs);
        rearLeftDrive.getConfigurator().apply(driveMotorConfigs);
        rearRightDrive.getConfigurator().apply(driveMotorConfigs);
    }

    // seconds from idle to max speed
    public void setDriveRampRate(double seconds) {
        // Open loop ramp rates
        driveMotorConfigs.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = seconds;

        // Closed loop ramp rates
        driveMotorConfigs.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = seconds;

        frontLeftDrive.getConfigurator().apply(driveMotorConfigs);
        frontRightDrive.getConfigurator().apply(driveMotorConfigs);
        rearLeftDrive.getConfigurator().apply(driveMotorConfigs);
        rearRightDrive.getConfigurator().apply(driveMotorConfigs);

        // frontLeftDrive.configOpenloopRamp(seconds);
        // frontRightDrive.configOpenloopRamp(seconds);
        // rearLeftDrive.configOpenloopRamp(seconds);
        // rearRightDrive.configOpenloopRamp(seconds);

        // frontLeftDrive.configClosedloopRamp(seconds);
        // frontRightDrive.configClosedloopRamp(seconds);
        // rearLeftDrive.configClosedloopRamp(seconds);
        // rearRightDrive.configClosedloopRamp(seconds);
    }

    public void setTurnCurrentLimit(double current) {
        turnMotorConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
        turnMotorConfigs.CurrentLimits.StatorCurrentLimit = current;
        // turnMotorConfigs.CurrentLimits.SupplyCurrentThreshold = current;
        // turnMotorConfigs.CurrentLimits.SupplyTimeThreshold = 0;

        frontLeftTurn.getConfigurator().apply(turnMotorConfigs);
        frontRightTurn.getConfigurator().apply(turnMotorConfigs);
        rearLeftTurn.getConfigurator().apply(turnMotorConfigs);
        rearRightTurn.getConfigurator().apply(turnMotorConfigs);
    }

    public void stopDrive() {
        frontLeftModule.stop();
        frontRightModule.stop();
        rearLeftModule.stop();
        rearRightModule.stop();

    }

    public void resetGyro() {
        navx.zeroYaw(); // This is always relative to which alliance station we are in.
        setGyroOffset(0.0);
    }

    public void resetGyro(double offset) {
        // Reset the gyro but set an offset
        navx.zeroYaw();
        setGyroOffset(offset);
    }

    public double getGyroYaw() {
        double angle = navx.getYaw() + gyroOffset;
        
        if (DriverStation.getAlliance()
            .orElse(DriverStation.Alliance.Red) == DriverStation.Alliance.Red) {
                angle += 180;
            }

        while (angle > 180) {
            angle -= 360;
        }
        while (angle < -180) {
            angle += 360;
        }
        return angle; // Navx is opposite sign from everything else
    }

    public double getGyroYawRate() {
        return navx.getRate();
    }

    public double getGyroOffset() {
        return gyroOffset;
    }

    public void setGyroOffset(double offset) {
        gyroOffset = offset;
    }

    public Pose2d getLatestSwervePose() {
        return latestSwervePose;
    }

    public void setModuleStates(SwerveModuleState[] states) {
        frontLeftModule.setState(states[0]);
        frontRightModule.setState(states[1]);
        rearLeftModule.setState(states[2]);
        rearRightModule.setState(states[3]);

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

    public void resetOdometry() {
        if (this.simpleOdometryReset) {
            swerveDrivePoseEstimator.resetPosition(
                    Rotation2d.fromDegrees(-getGyroYaw()), swerveDriveModulePositions,
                    new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(-getGyroYaw())));
        } else {
            double newGyroOffset = navx.getYaw() - resetPose.getRotation().getDegrees();
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
            fieldForPosePublishing.setRobotPose(this.latestSwervePose);
            SmartDashboard.putData("Odom Pose Field", fieldForPosePublishing);
        }
    }

    public int getOdometryResetCount() {
        return this.odometryResetCount;
    }

    public void scheduleOdometryReset() {
        this.simpleOdometryReset = true;
        this.odometryResetCount++;
    }

    public void scheduleOdometryReset(Pose2d initialPose) {
        this.simpleOdometryReset = false;
        this.resetPose = initialPose;
        this.odometryResetCount++;
    }

    public void onDisable() {
        setDriveNeutralMode(NeutralModeValue.Coast);    
        setTurnNeutralMode(NeutralModeValue.Coast);
        stopDrive();
    }

    public Command XWheels() {
        // return new SetSwerveAngle(this, 45, -45, -45, 45);
        return new frc.robot.commands.SetSwerveAngle(this, 45, -45, -45, 45);
    }

    public Command ResetOdometry() {
        return runOnce(() -> {
            this.scheduleOdometryReset();
        });
    }

    public Command ResetOdometryToPose(Pose2d initialPose) {
        return runOnce(() -> {
            this.scheduleOdometryReset(initialPose);
        });
    }

    public void doLimelightPoseUpdate() {
        double trustFactor = -1.0;
        LimelightHelpers.PoseEstimate limelightPoseEstimate = null;

        Pose2d adjustedRobotPose;
        double xAdjustment;
        double yAdjustment;

        // Poses are adjusted by a constant translation based on field calibration
        if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) {
            xAdjustment = Constants.Vision.fieldCalibrationSetting.redXAdjustmentMeters;
            yAdjustment = Constants.Vision.fieldCalibrationSetting.redYAdjustmentMeters;
        } else {
            xAdjustment = Constants.Vision.fieldCalibrationSetting.blueXAdjustmentMeters;
            yAdjustment = Constants.Vision.fieldCalibrationSetting.blueYAdjustmentMeters;
        }

        for (LimeLight limelight : limelightList) {
            limelightPoseEstimate = limelight.getLimelightMeasurement();
            
            // If we got a reading from the LimeLight, add it to the pose estimator with.
            // a trust factor calculated based on average distance from the targets seen.
            if (limelightPoseEstimate != null) {
                trustFactor = calculateVisionTrustFactor(limelightPoseEstimate, limelight);
                if (trustFactor > 0.0) {
                    if (DriverStation.isDisabled()) {
                        trustFactor /= 10.0; // Move to odometry fast under disable
                    }

                    // We see consistent errors based on Limelight readings, so this adjustment
                    // factor corrects for that so that our odometry values match actual robot 
                    // position on the field. This can be calibrated on the field at competitions.
                    adjustedRobotPose = new Pose2d(limelightPoseEstimate.pose.getX() + xAdjustment,
                            limelightPoseEstimate.pose.getY() + yAdjustment,
                            limelightPoseEstimate.pose.getRotation());

                    swerveDrivePoseEstimator.addVisionMeasurement(
                            adjustedRobotPose,
                            // timestamp is latency-corrected NT-referenced FPGA time
                            limelightPoseEstimate.timestampSeconds,
                            VecBuilder.fill(trustFactor, trustFactor, 9999999.9));
                }
            }
        }
    }

    public double calculateVisionTrustFactor(PoseEstimate visionEstimate, LimeLight ll) {
        // Anything less than 1m treat as 1M
        double distance = Math.max(visionEstimate.avgTagDist, 1.0);
        if (distance > ll.model.maxDistanceFromTag || this.lastDistanceMoved > ll.model.distanceMovedInCycleThreshold
                || Math.abs(this.getGyroYawRate()) >= ll.model.angularVelocityThreshold) {
            return -1; // Don't trust any readings further than this
        } else {
            return Math.pow(distance, 1.) * ll.model.trustFactor;
        }
    }

    // Move robot straight at a heading and speed
    public void moveRobotFrontBack(boolean forward, double velocity) {

        // Calculate the Swerve States
        double[] swerveStates;

        double y1 = velocity / Constants.DrivetrainConstants.swerveMaxSpeed;
        if (!forward) {
            y1 *= -1;
        }

        swerveStates = swerveController.calculate(0.0, y1, 0.0);

        // Get the Swerve Modules
        SwerveModuleFalconFalcon frontLeft = frontLeftModule;
        SwerveModuleFalconFalcon frontRight = frontRightModule;
        SwerveModuleFalconFalcon rearLeft = rearLeftModule;
        SwerveModuleFalconFalcon rearRight = rearRightModule;

        // Command the Swerve Modules
        frontLeft.setDriveVelocity(swerveStates[0], swerveStates[1]);
        frontRight.setDriveVelocity(swerveStates[2], swerveStates[3]);
        rearLeft.setDriveVelocity(swerveStates[4], swerveStates[5]);
        rearRight.setDriveVelocity(swerveStates[6], swerveStates[7]);
        // frontLeft.setDrive(velocity, 0);
        // frontRight.setDrive(velocity, 0);
        // rearLeft.setDrive(velocity, 0);
        // rearRight.setDrive(velocity, 0);
    }
    
    public void turnRobot( double heading) {

        // Calculate the Swerve States
        double[] swerveStates;

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


        swerveStates = swerveController.calculate(0.0, 0.0, x2);

        // Get the Swerve Modules
        SwerveModuleFalconFalcon frontLeft = frontLeftModule;
        SwerveModuleFalconFalcon frontRight = frontRightModule;
        SwerveModuleFalconFalcon rearLeft = rearLeftModule;
        SwerveModuleFalconFalcon rearRight = rearRightModule;

        // Command the Swerve Modules
        frontLeft.setDriveVelocity(swerveStates[0], swerveStates[1]);
        frontRight.setDriveVelocity(swerveStates[2], swerveStates[3]);
        rearLeft.setDriveVelocity(swerveStates[4], swerveStates[5]);
        rearRight.setDriveVelocity(swerveStates[6], swerveStates[7]);
    }

    // x2 = mDriveTrain.getGyroYaw() - targetAngle;

    // if (x2 > 180) {
    //     x2 -= 360;
    // } else if (x2 < -180) {
    //     x2 += 360;
    // }
    // if (Math.abs(x2 - targetAngle) > Constants.DrivetrainConstants.autoAngleThreshold) {
    //     x2 = x2 * Constants.DrivetrainConstants.driveRotationP;
    // } else {
    //     x2 = 0.0;
    // }

    // x2 = Math.min(x2, .90);
    // x2 = Math.max(x2, -.90);

    public boolean useLimeLightForOdometry() {
        return useLimelightOdometryUpdates;
    }

    public void setLimeLightOdometryUpdates(boolean isUpdating) {
        useLimelightOdometryUpdates = isUpdating;
    }

    public SwerveModuleFalconFalcon[] getSwerveModules() {
        SwerveModuleFalconFalcon[] modules = {
                frontLeftModule,
                frontRightModule,
                rearLeftModule,
                rearRightModule
        };
        return modules;
    }

    public void updateOdometryPose(SwerveModulePosition[] modulePositions) {
        this.odomReadingTesting = true;
        this.lastOdometryTranslation = this.latestSwervePose.getTranslation();
        this.latestSwervePose = this.swerveDrivePoseEstimator.updateWithTime(
                Timer.getFPGATimestamp(),
                Rotation2d.fromDegrees(-getGyroYaw()),
                modulePositions);
        this.lastDistanceMoved = lastOdometryTranslation.getDistance(this.latestSwervePose.getTranslation());
        fieldForPosePublishing.setRobotPose(this.latestSwervePose);
        SmartDashboard.putData("Odom Pose Field", fieldForPosePublishing);
    }

    public ChassisSpeeds getRobotChassisSpeeds() {
        SwerveModuleState[] moduleStates = {
                frontLeftModule.getState(),
                frontRightModule.getState(),
                rearLeftModule.getState(),
                rearRightModule.getState()
        };
        return swerveDriveKinematics.toChassisSpeeds(moduleStates);
    }

    public void driveRobotFromChassisSpeeds(ChassisSpeeds speeds) {
        SwerveModuleState[] states = swerveDriveKinematics.toSwerveModuleStates(speeds);
        setModuleStates(states);
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

   public void setEndgameTargetAngle(double angle) {
    endgameTargetAngle = angle;
   }

   public double getEndgameTargetAngle () {
    return endgameTargetAngle;
   }
   
   private double calculateDistanceFromSpeaker() {
    // goal changes depending on alliance
    if (DriverStation.getAlliance().orElse(DriverStation.Alliance.Red) == (DriverStation.Alliance.Red)) {
      return latestSwervePose.getTranslation()
          .getDistance(Constants.DrivetrainConstants.Field.redGoalTarget) * Constants.inchesPerMeter; // Inches
    } else {
      return latestSwervePose.getTranslation()
          .getDistance(Constants.DrivetrainConstants.Field.blueGoalTarget) * Constants.inchesPerMeter; // Inches
    }
  }
}
