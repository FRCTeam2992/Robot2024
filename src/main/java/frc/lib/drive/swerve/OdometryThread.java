package frc.lib.drive.swerve;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

public class OdometryThread extends Thread {
    private int refreshTimeoutFastMillis = Constants.DrivetrainConstants.odometryFastRefreshTimeoutMillis;
    private int refreshTimeoutSlowMillis = Constants.DrivetrainConstants.odometrySlowRefreshTimeoutMillis;

    private Drivetrain drivetrain;
    private SwerveModuleFalconFalcon[] modules;
    private boolean fastMode;
    private boolean cancelled;
    private double rioTimestamp;
    private SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];

    /* @class
     * OdometryThread class: performs odometry updates on a fast or slow interval.
     * 
     * Example:
     *   Odometry odometryThread = new Odometry(swerveModules);
     *   odometryThread.start();
     *   odometryThread.setFastMode();
     * 
     * Argument swerveModules - This should be an array of SwerveModuleFalconFalcon in the
     *   same order matching how SwerveModulePositions are ordered in Drivetrain:
     *     swerveDriveModulePositions[0] = frontLeftModule.getPosition();
     *     swerveDriveModulePositions[1] = frontRightModule.getPosition();
     *     swerveDriveModulePositions[2] = rearLeftModule.getPosition();
     *     swerveDriveModulePositions[3] = rearRightModule.getPosition();
     */
    public OdometryThread(Drivetrain subsystem) {
        super();
    
        this.drivetrain = subsystem;
        this.modules = this.drivetrain.getSwerveModules();
        this.rioTimestamp = Timer.getFPGATimestamp();
        this.fastMode = false;
        this.cancelled = false;
    }

    public void run() {

        System.out.println("<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< Odom Thread Started");

        int n;

        while (!this.cancelled) {
            this.rioTimestamp = Timer.getFPGATimestamp();

            for (n = 0; n < this.modules.length; n++) {
                this.modulePositions[n] = this.modules[n].getPosition();
            }
            drivetrain.updateOdometryPose(this.modulePositions);

            Timer.delay(
                Math.max(0.0, this.rioTimestamp + refreshTimeoutSeconds() - Timer.getFPGATimestamp())
            );
        }
    }

    private double refreshTimeoutSeconds() {
        if (this.fastMode) {
            return this.refreshTimeoutFastMillis / 1000.0;
        }
        return this.refreshTimeoutSlowMillis / 1000.0;
    }

    public void setFastMode() {
        this.fastMode = true;
    }

    public void setSlowMode() {
        this.fastMode = false;
    }

    public void cancel() {
        this.cancelled = true;
    }
}
