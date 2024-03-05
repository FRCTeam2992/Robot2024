// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;

import javax.print.attribute.standard.Media;

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
// import com.ctre.phoenix.motorcontrol.can.TalonFX;
// import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.kauailabs.navx.frc.AHRS;
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

// import frc.lib.NavX.AHRS;

import frc.lib.drive.swerve.SwerveController;
import frc.lib.drive.swerve.SwerveModuleFalconFalcon;
import frc.lib.vision.LimeLight;
import frc.lib.vision.LimeLight.CoordinateSpace;
import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleArrayLogEntry;
import edu.wpi.first.util.datalog.IntegerLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.SPI;
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
    public final LimeLight limeLightCameraBack;
    public final ArrayList<LimeLight> limelightList;
    public double limeLightBlendedLatency = 0.0;
    private double[] limelightBackBotPose;
    private boolean isUpdatingLimelightOdometry = true;
    private double limelightTotalArea = 0.0;
    private int limelightCalculationsCount = 0;
    private MedianFilter limelightXMedianFilter;
    private MedianFilter limelightYMedianFilter;
    private MedianFilter limelightAngleMedianFilter;

    private DataLog mDataLog;

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
    private boolean inSlowMode = false;
    private boolean doFieldOrient = true;
    private boolean autoRotate = false;
    private boolean loadingMode = false;
    private boolean useLimelightOdometryUpdates = false;
    private int odometryResetCount = 0;
    private int lastOdometryResetCount = -1;
    private boolean simpleOdometryReset = true;
    public Pose2d resetPose = Constants.DrivetrainConstants.zeroPose;

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

        setDriveCurrentLimit(40.0, 40.0);
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
        limeLightCameraBack = new LimeLight("limelight-back");
        limelightBackBotPose = new double[7];

        limelightList = new ArrayList<LimeLight>();
        limelightList.add(limeLightCameraBack);

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
                MatBuilder.fill(Nat.N3(), Nat.N1(), 0.002, 0.002, 0.01),
                // Global measurement standard deviations. X, Y, and theta.
                MatBuilder.fill(Nat.N3(), Nat.N1(), 0.01, 0.01, .9999));

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
                        3.0, // Max module speed, in m/s
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
                if (useLimeLightForOdometry()) {
                    calculateBlendedVisionPose();
                    if (latestVisionPoseValid) {
                        swerveDrivePoseEstimator.addVisionMeasurement(
                                latestVisionPose, Timer.getFPGATimestamp() - limeLightBlendedLatency / 1000);
                    }
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
            }

            if (Constants.debugDashboard) {
                SmartDashboard.putBoolean("Limelight Back Pose OK?", limelightBackBotPose.length <= 7);
                SmartDashboard.putBoolean("Limelight Seeing Target?", limeLightCameraBack.getTargetID() != -1);
                SmartDashboard.putBoolean("Latest Vision Pose OK?", latestVisionPose != null);
                SmartDashboard.putNumber("Limelight Calculations", limelightCalculationsCount);

                if (limelightBackBotPose != null) {
                    SmartDashboard.putNumber("Limelight Back Pose X (m)",
                            limelightBackBotPose[0]);
                    SmartDashboard.putNumber("Limelight Back Pose Y (m)",
                            limelightBackBotPose[1]);
                    SmartDashboard.putNumber("Limelight Back Pose Yaw (deg)",
                            limelightBackBotPose[5]);
                    // SmartDashboard.putNumber("Limelight Back Pose Latency (ms)",
                    // limelightBackBotPose[6]);
                    SmartDashboard.putNumber("Limelight Back Tid",
                            limeLightCameraBack.getTargetID());
                    SmartDashboard.putNumber("Limelight Back Ta",
                            limeLightCameraBack.getTargetArea());
                }

                if (latestVisionPose != null) {
                    SmartDashboard.putNumber("Limelight Blended X (m)", latestVisionPose.getX());
                    SmartDashboard.putNumber("Limelight Blended Y (m)", latestVisionPose.getY());
                    SmartDashboard.putNumber("Limelight Blended Theta", latestVisionPose.getRotation().getDegrees());
                    SmartDashboard.putNumber("Limelight Blended FPGATime", Timer.getFPGATimestamp());
                    SmartDashboard.putNumber("Limelight Blended Latency", limeLightBlendedLatency);
                }

                SmartDashboard.putBoolean("Updating odometry from LimeLight", isUpdatingLimelightOdometry);
                SmartDashboard.putBoolean("Latest Vision Pose Valid?", latestVisionPoseValid);
                SmartDashboard.putNumber("LimeLight Total Area", limelightTotalArea);

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

        driveMotorConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;
        driveMotorConfigs.CurrentLimits.SupplyCurrentLimit = currentLimit;
        driveMotorConfigs.CurrentLimits.SupplyCurrentThreshold = triggerCurrent;
        driveMotorConfigs.CurrentLimits.SupplyTimeThreshold = 0;

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
        turnMotorConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;
        turnMotorConfigs.CurrentLimits.SupplyCurrentLimit = current;
        turnMotorConfigs.CurrentLimits.SupplyCurrentThreshold = current;
        turnMotorConfigs.CurrentLimits.SupplyTimeThreshold = 0;

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
        navx.zeroYaw(); // TODO: Handle red/blue needed?
        setGyroOffset(0.0);
    }

    public void resetGyro(double offset) {
        // Reset the gyro but set an offset
        navx.zeroYaw();
        setGyroOffset(offset);
    }

    public double getGyroYaw() {
        double angle = navx.getYaw() + gyroOffset;
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

    public void calculateBlendedVisionPose() {

        if (!DriverStation.getAlliance().isPresent()) {
            limeLightBlendedLatency = 0.0;
            latestVisionPoseValid = false;
            limelightXMedianFilter.reset();
            limelightYMedianFilter.reset();
            limelightAngleMedianFilter.reset();
            return;
        }

        SmartDashboard.putString("Alliance Color", getAllianceCoordinateSpace().toString());
        limelightBackBotPose = limeLightCameraBack.getBotPose(getAllianceCoordinateSpace());

        if (limelightBackBotPose != null && limeLightCameraBack.getTargetID() != -1) {
            limelightTotalArea = limeLightCameraBack.getTargetArea();

            limelightCalculationsCount++;
            latestVisionPoseValid = true;

            if (limelightTotalArea <= 0.05) {
                limeLightBlendedLatency = 0.0;
                latestVisionPoseValid = false;
                limelightXMedianFilter.reset();
                limelightYMedianFilter.reset();
                limelightAngleMedianFilter.reset();
                return;
            }

            isUpdatingLimelightOdometry = true;
            limeLightBlendedLatency = limelightBackBotPose[6];

            latestVisionPose = new Pose2d(
                    limelightXMedianFilter.calculate(limelightBackBotPose[0]),
                    limelightYMedianFilter.calculate(limelightBackBotPose[1]),
                    Rotation2d.fromDegrees(limelightAngleMedianFilter.calculate(limelightBackBotPose[5])));
            isUpdatingLimelightOdometry = false;
        } else {
            limelightXMedianFilter.reset();
            limelightYMedianFilter.reset();
            limelightAngleMedianFilter.reset();
            latestVisionPoseValid = false;
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
    }

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
        odomReadingTesting = true;
        this.latestSwervePose = this.swerveDrivePoseEstimator.updateWithTime(
                Timer.getFPGATimestamp(),
                Rotation2d.fromDegrees(-getGyroYaw()),
                modulePositions);
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
}
