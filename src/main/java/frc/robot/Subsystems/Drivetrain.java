// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import java.util.ArrayList;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CANcoderConfigurator;
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
import com.pathplanner.lib.path.PathPlannerTrajectory;

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
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleArrayLogEntry;
import edu.wpi.first.util.datalog.IntegerLogEntry;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Commands.SetSwerveAngle;

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
    // public final LimeLight limeLightCameraBack;
    public final LimeLight limeLightCameraRight;
    public final LimeLight limeLightCameraLeft;
    private double limeLightBlendedLatency = 0.0;
    public ArrayList<LimeLight> limelightList;

    private DataLog mDataLog;

    // private StringLogEntry limelight11JsonLog;
    // private StringLogEntry limelight12JsonLog;
    // private StringLogEntry limelight13JsonLog;

    // private DoubleArrayLogEntry ll11BotposeFieldSpaceLog;
    // private DoubleArrayLogEntry ll12BotposeFieldSpaceLog;
    // private DoubleArrayLogEntry ll13BotposeFieldSpaceLog;
    private DoubleArrayLogEntry llBackBotposeBlueLog;
    private DoubleArrayLogEntry llRightBotposeBlueLog;
    private DoubleArrayLogEntry llLeftBotposeBlueLog;
    private DoubleArrayLogEntry llBackBotposeRedLog;
    private DoubleArrayLogEntry llRightBotposeRedLog;
    private DoubleArrayLogEntry llLeftBotposeRedLog;
    private IntegerLogEntry llBackTargetIDLog;
    private IntegerLogEntry llRightTargetIDLog;
    private IntegerLogEntry llLeftTargetIDLog;
    // private double[] limelightBackBotPose;
    private double[] limelightRightBotPose;
    private double[] limelightLeftBotPose;

    // Robot Gyro
    public AHRS navx;
    public double gyroOffset = 0.0;

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

    // public PathPlannerTrajectory testPath;
    public PathPlannerTrajectory driveStraight;
    public PathPlannerTrajectory curvePath;
    public PathPlannerTrajectory testPath;

    // State Variables
    private boolean inSlowMode = false;
    private boolean doFieldOreint = true;
    private boolean scoringMode = false;
    private boolean loadingMode = false;
    private boolean useLimelightOdometryUpdates;

    private int dashboardCounter = 0;

    public Drivetrain() {

        // Motor Inits
        frontLeftDrive = new TalonFX(Constants.DrivetrainConstants.CanIDs.frontLeftDrive);
        initTalonFX(frontLeftDrive, driveMotorConfigs, "CounterClockwise_Positive");

        frontLeftTurn = new TalonFX(Constants.DrivetrainConstants.CanIDs.frontLeftTurn);
        initTalonFX(frontLeftTurn, turnMotorConfigs, "Clockwise_Positive");

        frontRightDrive = new TalonFX(Constants.DrivetrainConstants.CanIDs.frontRightDrive);
        initTalonFX(frontRightDrive, driveMotorConfigs,"CounterClockwise_Positive");

        frontRightTurn = new TalonFX(Constants.DrivetrainConstants.CanIDs.frontRightTurn);
        initTalonFX(frontRightTurn, turnMotorConfigs, "Clockwise_Positive");

        rearRightDrive = new TalonFX(Constants.DrivetrainConstants.CanIDs.rearRightDrive);
        initTalonFX(rearRightDrive, driveMotorConfigs,"CounterClockwise_Positive");

        rearRightTurn = new TalonFX(Constants.DrivetrainConstants.CanIDs.rearRightTurn);
        initTalonFX(rearRightTurn, turnMotorConfigs,"Clockwise_Positive");

        rearLeftDrive = new TalonFX(Constants.DrivetrainConstants.CanIDs.rearLeftDrive);
        initTalonFX(rearLeftDrive, driveMotorConfigs,"CounterClockwise_Positive");

        rearLeftTurn = new TalonFX(Constants.DrivetrainConstants.CanIDs.rearLeftTurn);
        initTalonFX(rearLeftTurn, turnMotorConfigs,"Clockwise_Positive");

        driveMotorConfigs = new TalonFXConfiguration();
        turnMotorConfigs = new TalonFXConfiguration();


        frontRightEncoder = new CANcoder(Constants.DrivetrainConstants.CanIDs.frontRightEncoder);
        // initCANCoder(frontRightEncoder, AbsoluteSensorRange.Signed_PlusMinus180, true);
        initCANCoder(frontRightEncoder, "Signed_PlusMinusHalf", "Clockwise_Positive");

        frontLeftEncoder = new CANcoder(Constants.DrivetrainConstants.CanIDs.frontLeftEncoder);
        initCANCoder(frontLeftEncoder, "Signed_PlusMinusHalf", "Clockwise_Positive");

        rearRightEncoder = new CANcoder(Constants.DrivetrainConstants.CanIDs.rearRightEncoder);
        initCANCoder(rearRightEncoder, "Signed_PlusMinusHalf", "Clockwise_Positive");

        rearLeftEncoder = new CANcoder(Constants.DrivetrainConstants.CanIDs.rearLeftEncoder);
        initCANCoder(rearLeftEncoder,"Signed_PlusMinusHalf", "Clockwise_Positive");

        encoderConfigs = new CANcoderConfiguration();


        setDriveNeutralMode("Coast");
        setTurnNeutralMode("Brake");

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

        // Load Motion Paths
        loadMotionPaths();

        // Limelight
        limelightList = new ArrayList<>();
        // limeLightCameraBack = new LimeLight("limelight-back");
        // limelightList.add(limeLightCameraBack);
        limeLightCameraRight = new LimeLight("limelight-right");
        limelightList.add(limeLightCameraRight);
        limeLightCameraLeft = new LimeLight("limelight-left");
        limelightList.add(limeLightCameraLeft);
        // limelightBackBotPose = new double[7];
        limelightRightBotPose = new double[7];
        limelightLeftBotPose = new double[7];

        if (Constants.dataLogging) {
            mDataLog = DataLogManager.getLog();

            // limelight11JsonLog = new StringLogEntry(mDataLog, "/ll/eleven/json");
            // limelight12JsonLog = new StringLogEntry(mDataLog, "/ll/twelve/json");
            // limelight13JsonLog = new StringLogEntry(mDataLog, "/ll/thirteen/json");

            // ll11BotposeFieldSpaceLog = new DoubleArrayLogEntry(mDataLog,
            // "/ll/eleven/botpose_field");
            // ll12BotposeFieldSpaceLog = new DoubleArrayLogEntry(mDataLog,
            // "/ll/twelve/botpose_field");

            llBackBotposeBlueLog = new DoubleArrayLogEntry(mDataLog, "/ll/eleven/botpose_blue");
            llRightBotposeBlueLog = new DoubleArrayLogEntry(mDataLog, "/ll/twelve/botpose_blue");
            llLeftBotposeBlueLog = new DoubleArrayLogEntry(mDataLog, "/ll/thirteen/botpose_blue");

            llBackBotposeRedLog = new DoubleArrayLogEntry(mDataLog, "/ll/eleven/botpose_red");
            llRightBotposeRedLog = new DoubleArrayLogEntry(mDataLog, "/ll/twelve/botpose_red");
            llLeftBotposeRedLog = new DoubleArrayLogEntry(mDataLog, "/ll/thirteen/botpose_red");

            llBackTargetIDLog = new IntegerLogEntry(mDataLog, "/ll/eleven/target_id");
            llRightTargetIDLog = new IntegerLogEntry(mDataLog, "/ll/twelve/target_id");
            llLeftTargetIDLog = new IntegerLogEntry(mDataLog, "/ll/thirteen/target_id");

            // Controll Requests
            percentControlRequest = new DutyCycleOut(0.0);
            velocityControlRequest = new VelocityDutyCycle(0.0);

        }

        // robot gyro initialization
        navx = new AHRS(SPI.Port.kMXP, (byte) 50);

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
                // Rotation2d.fromDegrees(navx.getYaw()),
                Rotation2d.fromDegrees(getGyroYaw()),
                swerveDriveModulePositions,
                new Pose2d(0.0, 0.0, new Rotation2d()),
                // State measurement standard deviations. X, Y, theta.
                new MatBuilder<>(Nat.N3(), Nat.N1()).fill(0.02, 0.02, 0.01),
                // Global measurement standard deviations. X, Y, and theta.
                new MatBuilder<>(Nat.N3(), Nat.N1()).fill(0.1, 0.1, .9999));
    }

    private void initTalonFX(TalonFX motorContollerName, TalonFXConfiguration configs, String motorDirection) {
        configs.MotorOutput.Inverted = InvertedValue.valueOf(motorDirection);
        motorContollerName.getConfigurator().apply(configs);
    }

    private void initCANCoder(CANcoder CANCoderName, String sensorRange, String sensorDirection) {
        encoderConfigs.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.valueOf(sensorRange);
        encoderConfigs.MagnetSensor.SensorDirection = SensorDirectionValue.valueOf(sensorDirection);
        CANCoderName.getConfigurator().apply(encoderConfigs);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        swerveDriveModulePositions[0] = frontLeftModule.getPosition();
        swerveDriveModulePositions[1] = frontRightModule.getPosition();
        swerveDriveModulePositions[2] = rearLeftModule.getPosition();
        swerveDriveModulePositions[3] = rearRightModule.getPosition();

        latestSwervePose = swerveDrivePoseEstimator.updateWithTime(
                Timer.getFPGATimestamp(),
                Rotation2d.fromDegrees(-getGyroYaw()),
                swerveDriveModulePositions);

        // limelightBackBotPose =
        // limeLightCameraBack.getBotPose(getAllianceCoordinateSpace());
        limelightRightBotPose = limeLightCameraRight.getBotPose(getAllianceCoordinateSpace());
        limelightLeftBotPose = limeLightCameraLeft.getBotPose(getAllianceCoordinateSpace());

        if (Constants.dataLogging) {
            // limelight11JsonLog.append(limeLightCamera11.getLimelightJson());
            // limelight12JsonLog.append(limeLightCamera12.getLimelightJson());
            // limelight13JsonLog.append(limeLightCamera13.getLimelightJson());

            // ll11BotposeFieldSpaceLog.append(limeLightCamera11.getBotPose(CoordinateSpace.Field));
            // ll12BotposeFieldSpaceLog.append(limeLightCamera12.getBotPose(CoordinateSpace.Field));
            // ll13BotposeFieldSpaceLog.append(limeLightCamera13.getBotPose(CoordinateSpace.Field));

            // llBackBotposeBlueLog.append(limeLightCameraBack.getBotPose(CoordinateSpace.Blue));
            llRightBotposeBlueLog.append(limeLightCameraRight.getBotPose(CoordinateSpace.Blue));
            llLeftBotposeBlueLog.append(limeLightCameraLeft.getBotPose(CoordinateSpace.Blue));

            // llBackBotposeRedLog.append(limeLightCameraBack.getBotPose(CoordinateSpace.Red));
            llRightBotposeRedLog.append(limeLightCameraRight.getBotPose(CoordinateSpace.Red));
            llLeftBotposeRedLog.append(limeLightCameraLeft.getBotPose(CoordinateSpace.Red));

            // llBackTargetIDLog.append(limeLightCameraBack.getTargetID());
            llRightTargetIDLog.append(limeLightCameraRight.getTargetID());
            llLeftTargetIDLog.append(limeLightCameraLeft.getTargetID());
        }

        if (dashboardCounter++ >= 5) {
            if (Constants.debugDashboard) {
                SmartDashboard.putNumber("Odometry Rotation (deg)", latestSwervePose.getRotation().getDegrees());
                SmartDashboard.putNumber("Odometry X (in)", (latestSwervePose.getX() * (100 / 2.54)));
                SmartDashboard.putNumber("Odometry Y (in)", (latestSwervePose.getY() * (100 / 2.54)));
                SmartDashboard.putNumber("Odometry X (m)", latestSwervePose.getX());
                SmartDashboard.putNumber("Odometry Y (m)", latestSwervePose.getY());
            }
            SmartDashboard.putNumber("front left encoder", frontLeftModule.getEncoderAngle());
            SmartDashboard.putNumber("front right encoder", frontRightModule.getEncoderAngle());
            SmartDashboard.putNumber("back left encoder", rearLeftModule.getEncoderAngle());
            SmartDashboard.putNumber("back right encoder", rearRightModule.getEncoderAngle());

            SmartDashboard.putNumber("Gyro Yaw (raw deg)", navx.getYaw());
            SmartDashboard.putNumber("Gyro Yaw (adj deg)", getGyroYaw());
            SmartDashboard.putNumber("Robot Gyro Pitch (raw deg)", getRobotPitch()); // Navx Roll

            // if (limeLightCameraBack.getTargetID() > 0) {
            // if (Constants.debugDashboard) {
            // SmartDashboard.putNumber("Limelight Pose X (m)", limelightBackBotPose[0]);
            // SmartDashboard.putNumber("Limelight Pose Y (m)", limelightBackBotPose[1]);
            // SmartDashboard.putNumber("Limelight Pose Yaw (deg)",
            // limelightBackBotPose[5]);
            // // SmartDashboard.putNumber("Limelight Pose Latency (ms)",
            // // limelight11BotPose[6]);
            // }
            // }

            if (Constants.debugDashboard) {
                // if (limeLightCameraBack.getTargetID() > 0) {
                // if (Constants.debugDashboard) {
                // SmartDashboard.putNumber("Limelight Back Pose X (m)",
                // limelightBackBotPose[0]);
                // SmartDashboard.putNumber("Limelight Back Pose Y (m)",
                // limelightBackBotPose[1]);
                // SmartDashboard.putNumber("Limelight Back Pose Yaw (deg)",
                // limelightBackBotPose[5]);
                // SmartDashboard.putNumber("Limelight Back Pose Latency (ms)",
                // limelightBackBotPose[6]);
                // }
                // }
                if (limeLightCameraRight.getTargetID() > 0) {
                    if (Constants.debugDashboard) {
                        SmartDashboard.putNumber("Limelight Right Pose X (m)",
                                limelightRightBotPose[0]);
                        SmartDashboard.putNumber("Limelight Right Pose Y (m)",
                                limelightRightBotPose[1]);
                        SmartDashboard.putNumber("Limelight Right Pose Yaw (deg)",
                                limelightRightBotPose[5]);
                        SmartDashboard.putNumber("Limelight Right Pose Latency (ms)",
                                limelightRightBotPose[6]);
                    }
                }
                if (limeLightCameraLeft.getTargetID() > 0) {
                    if (Constants.debugDashboard) {
                        SmartDashboard.putNumber("Limelight Left Pose X (m)",
                                limelightLeftBotPose[0]);
                        SmartDashboard.putNumber("Limelight Left Pose Y (m)",
                                limelightLeftBotPose[1]);
                        SmartDashboard.putNumber("Limelight Left Pose Yaw (deg)",
                                limelightLeftBotPose[5]);
                        SmartDashboard.putNumber("Limelight Left Pose Latency (ms)",
                                limelightLeftBotPose[6]);
                    }
                }

                // SmartDashboard.putNumber("Limelight Back Tid",
                // limeLightCameraBack.getTargetID());
                // SmartDashboard.putNumber("Limelight Back Ta",
                // limeLightCameraBack.getTargetArea());
                SmartDashboard.putNumber("Limelight Right Tid",
                        limeLightCameraRight.getTargetID());
                SmartDashboard.putNumber("Limelight Right Ta",
                        limeLightCameraRight.getTargetArea());
                SmartDashboard.putNumber("Limelight Left Tid",
                        limeLightCameraLeft.getTargetID());
                SmartDashboard.putNumber("Limelight Left Ta",
                        limeLightCameraLeft.getTargetArea());

                if (latestVisionPose != null) {
                    SmartDashboard.putNumber("Limelight Blended X (m)", latestVisionPose.getX());
                    SmartDashboard.putNumber("Limelight Blended Y (m)", latestVisionPose.getY());
                    SmartDashboard.putNumber("Limelight Blended Theta", latestVisionPose.getRotation().getDegrees());
                    SmartDashboard.putNumber("Limelight Blended FPGATime", Timer.getFPGATimestamp());
                    SmartDashboard.putNumber("Limelight Blended Latency", limeLightBlendedLatency);
                }
            }
            dashboardCounter = 0;
        }

        if (useLimelightOdometryUpdates) {
            calculateBlendedVisionPose();
            if (latestVisionPoseValid) {
                swerveDrivePoseEstimator.addVisionMeasurement(latestVisionPose,
                        Timer.getFPGATimestamp() - limeLightBlendedLatency / 1000);
            }
        }

        // if (this.mRobotState.useLimelightOdometryUpdates) {
        //     calculateBlendedVisionPose();
        //     if (latestVisionPoseValid) {
        //         swerveDrivePoseEstimator.addVisionMeasurement(latestVisionPose,
        //                 Timer.getFPGATimestamp() - limeLightBlendedLatency / 1000);
        //     }
        // }
    }

    public CoordinateSpace getAllianceCoordinateSpace() {
        if(!DriverStation.getAlliance().isPresent()) {
            return CoordinateSpace.Field;
        }
        if (DriverStation.getAlliance().orElse(DriverStation.Alliance.Red) == DriverStation.Alliance.Red) {
            return CoordinateSpace.Red;
        }
        return CoordinateSpace.Blue;
    }

    public void setDriveNeutralMode(String mode) {
        driveMotorConfigs.MotorOutput.NeutralMode = NeutralModeValue.valueOf(mode);
        
        frontLeftDrive.getConfigurator().apply(driveMotorConfigs);
        frontRightDrive.getConfigurator().apply(driveMotorConfigs);
        rearLeftDrive.getConfigurator().apply(driveMotorConfigs);
        rearRightDrive.getConfigurator().apply(driveMotorConfigs);

    }

    public void setTurnNeutralMode(String mode) {
        turnMotorConfigs.MotorOutput.NeutralMode = NeutralModeValue.valueOf(mode);
        
        frontLeftTurn.getConfigurator().apply(driveMotorConfigs);
        frontRightTurn.getConfigurator().apply(driveMotorConfigs);
        rearLeftTurn.getConfigurator().apply(driveMotorConfigs);
        rearRightTurn.getConfigurator().apply(driveMotorConfigs);
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

        // frontLeftDrive
        //         .configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, currentLimit, triggerCurrent, 0));
        // frontRightDrive
        //         .configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, currentLimit, triggerCurrent, 0));
        // rearLeftDrive
        //         .configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, currentLimit, triggerCurrent, 0));
        // rearRightDrive
        //         .configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, currentLimit, triggerCurrent, 0));
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
        driveMotorConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;
        driveMotorConfigs.CurrentLimits.SupplyCurrentLimit = current;
        driveMotorConfigs.CurrentLimits.SupplyCurrentThreshold = current;
        driveMotorConfigs.CurrentLimits.SupplyTimeThreshold = 0;

        frontLeftTurn.getConfigurator().apply(driveMotorConfigs);
        frontRightTurn.getConfigurator().apply(driveMotorConfigs);
        rearLeftTurn.getConfigurator().apply(driveMotorConfigs);
        rearRightTurn.getConfigurator().apply(driveMotorConfigs);

        // frontLeftTurn.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, current, current, 0));
        // frontRightTurn.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, current, current, 0));
        // rearLeftTurn.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, current, current, 0));
        // rearRightTurn.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, current, current, 0));
    }

    public void stopDrive() {
        frontLeftModule.stop();
        frontRightModule.stop();
        rearLeftModule.stop();
        rearRightModule.stop();

    }

    public void resetOdometry() {
        swerveDrivePoseEstimator.resetPosition(Rotation2d.fromDegrees(-getGyroYaw()), swerveDriveModulePositions,
                new Pose2d(0.0, 0.0, new Rotation2d()));
    }

    public void resetOdometryToPose(Pose2d initialPose) {
        setGyroOffset(navx.getYaw() - initialPose.getRotation().getDegrees());
        swerveDrivePoseEstimator.resetPosition(Rotation2d.fromDegrees(-getGyroYaw()), swerveDriveModulePositions,
                initialPose);
        latestSwervePose = initialPose;
    }

    public double getGyroYaw() {
        double angle = navx.getYaw() + gyroOffset;
        while (angle > 360) {
            angle -= 360;
        }
        while (angle < 0) {
            angle += 360;
        }
        return angle;
    }

    public void setGyroOffset(double offset) {
        gyroOffset = offset;
    }

    public double getRobotPitch() {
        return -1 * (navx.getRoll() + Constants.DrivetrainConstants.gyroRollOffset);
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

    public boolean getDoFieldOreint() {
        return doFieldOreint;
    }

    public void setDoFieldOreint(boolean disableFieldOreint) {
        this.doFieldOreint = disableFieldOreint;
    }

    public boolean isScoringMode() {
        return scoringMode;
    }

    public void setScoringMode(boolean scoringMode) {
        this.scoringMode = scoringMode;
    }

    public boolean isLoadingMode() {
        return loadingMode;
    }

    public void setLoadingMode(boolean loadingMode) {
        this.loadingMode = loadingMode;
    }

    public void onDisable() {
        setDriveNeutralMode("Brake");
        setTurnNeutralMode("Brake");
        stopDrive();
    }

    private void loadMotionPaths() {
        // testPath = PathPlanner.loadPath("Test Path", new PathConstraints(2, 1.5));
        // driveStraight = PathPlanner.loadPath("DriveStraight", new PathConstraints(.5, .5));
        // curvePath = PathPlanner.loadPath("CurvePath", new PathConstraints(.5, .5));
        // testPath = PathPlanner.loadPath("TestPath", new PathConstraints(.5, .5));
        // NOTE: Motion paths used in autonomous sequences have been moved to
        // frc.lib.autonomous.AutonomousTrajectory.
    }

    public Command XWheels() {
        // return new SetSwerveAngle(this, 45, -45, -45, 45);
        return new frc.robot.Commands.SetSwerveAngle(this, 45, -45, -45, 45);
    }

    public Command ResetOdometry() {
        return runOnce(() -> {
            resetOdometry();
        });
    }

    private void calculateBlendedVisionPose() {
        double sumX = 0.0;
        double sumY = 0.0;
        double sumTheta = 0.0;
        double sumLatency = 0.0;
        double totalArea = 0.0;

        if (!DriverStation.getAlliance().isPresent()) {
            limeLightBlendedLatency = 0.0;
            latestVisionPoseValid = false;
            return;
        } 
        // if (limeLightCameraBack.getTargetID() != -1) {
        // double Ta = limeLightCameraBack.getTargetArea();
        // totalArea += Ta;
        // sumX += Ta * limelightBackBotPose[0];
        // sumY += Ta * limelightBackBotPose[1];
        // sumTheta += Ta * limelightBose[5];
        // sumLatency += Ta * limelightBackBotPose[6];
        // }
        if (limeLightCameraRight.getTargetID() != -1) {
            double Ta = limeLightCameraRight.getTargetArea();
            totalArea += Ta;
            sumX += Ta * limelightRightBotPose[0];
            sumY += Ta * limelightRightBotPose[1];
            sumTheta += Ta * limelightRightBotPose[5];
            sumLatency += Ta * limelightRightBotPose[6];
        }
        if (limeLightCameraLeft.getTargetID() != -1) {
            double Ta = limeLightCameraLeft.getTargetArea();
            totalArea += Ta;
            sumX += Ta * limelightLeftBotPose[0];
            sumY += Ta * limelightLeftBotPose[1];
            sumTheta += Ta * limelightLeftBotPose[5];
            sumLatency += Ta * limelightLeftBotPose[6];
        }

        SmartDashboard.putNumber("Limelight TotalArea", totalArea);
        if (totalArea == 0.0) {
            limeLightBlendedLatency = 0.0;
            latestVisionPoseValid = false;
            return;
        }
        limeLightBlendedLatency = sumLatency / totalArea;
        latestVisionPoseValid = true;
        latestVisionPose = new Pose2d(sumX / totalArea, sumY / totalArea, Rotation2d.fromDegrees(sumTheta / totalArea));
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

    public void setLimeLightOdometryUpdates(boolean isUpdating){
        useLimelightOdometryUpdates = isUpdating;
    }
}
