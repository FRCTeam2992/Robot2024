package frc.lib.drive.swerve;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class SwerveModuleFalconFalcon {

    // Saved Variables
    private TalonFX driveMotor;
    private TalonFX turnMotor;
    private CANcoder encoderInput;
    private double encoderOffset;
    private double wheelDiameter;
    private double wheelGearRatio;
    private double maxDriveSpeed;
    private PIDController turnPID;
    private DutyCycleOut percentControlRequest;
    private VelocityDutyCycle velocityControlRequest;

    public SwerveModuleFalconFalcon(com.ctre.phoenix6.hardware.TalonFX driveMotor, com.ctre.phoenix6.hardware.TalonFX turnMotor, CANcoder encoder,
            double encoderOffset, PIDController turnPID, double wheelDiameter, double wheelGearRatio,
            double maxDriveSpeed, DutyCycleOut percentOut, VelocityDutyCycle velocityOut) {
        // Saved Variables
        this.driveMotor = driveMotor;
        this.turnMotor = turnMotor;
        this.encoderInput = encoder;
        this.encoderOffset = encoderOffset;
        this.turnPID = turnPID;
        this.wheelDiameter = wheelDiameter;
        this.wheelGearRatio = wheelGearRatio;
        this.maxDriveSpeed = maxDriveSpeed;
        this.percentControlRequest = percentOut;
        this.velocityControlRequest = velocityOut;
    }

    public void setDriveSpeed(double speed) {
        percentControlRequest.Output = speed;
        driveMotor.setControl(percentControlRequest);   
    }

    public void setTurnSpeed(double speed) {
        percentControlRequest.Output = speed;
        turnMotor.setControl(percentControlRequest);
    }

    public void stop() {
        setDriveSpeed(0.0);
        setTurnSpeed(0.0);
    }

    public void setTurnAngle(double degrees) {
        degrees = Math.min(Math.max(degrees, -180.0), 180.0);
        setTurnSpeed(turnPID.calculate(getEncoderAngle(), degrees));
    }

    public void setVelocityMeters(double speed) {
        double RPM = (speed * wheelGearRatio * 60) / (wheelDiameter * Math.PI);
        velocityControlRequest.Velocity = RPM / 60;
        driveMotor.setControl(velocityControlRequest);
    }

    public void setDrive(double speed, double angle) {
        if (Math.abs(getEncoderAngle() - angle) > 90.0) {
            if (angle > 0) {
                angle -= 180.0;
            } else {
                angle += 180.0;
            }

            speed = -speed;
        }

        setDriveSpeed(speed);
        setTurnAngle(angle);
    }

    public void setDriveVelocity(double speedPercent, double angle) {
        double speed = speedPercent * maxDriveSpeed;

        if (Math.abs(getEncoderAngle() - angle) > 90.0) {
            if (angle > 0) {
                angle -= 180.0;
            } else {
                angle += 180.0;
            }

            speed = -speed;
        }

        setVelocityMeters(speed);
        setTurnAngle(angle);
    }

    public boolean atAngle() {
        return turnPID.atSetpoint();
    }

    public double getEncoderAngle() {
        double tempAngle = encoderInput.getAbsolutePosition().getValue() * 360 - encoderOffset;
        // double tempAngle = encoderInput.getAbsolutePosition() - encoderOffset;

        // Not sure if -180 adjust needed. Not sure why this was here before
        // tempAngle += 180.0;

        if (tempAngle < -180.0) {
            tempAngle += 360.0;
        } else if (tempAngle > 180.0) {
            tempAngle -= 360.0;
        }

        return -tempAngle;
    }

    public double getWheelSpeedMeters() {
        double RPM = (driveMotor.getVelocity().getValue() * 60);
        // double RPM = (driveMotor.getSelectedSensorVelocity() * 600) / 2048;

        double speed = (RPM * wheelDiameter * Math.PI) / (wheelGearRatio * 60);

        return speed;
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getWheelSpeedMeters(), Rotation2d.fromDegrees(getEncoderAngle()));
    }

    public void setState(SwerveModuleState state) {
        double angle = state.angle.getDegrees();
        double speed = state.speedMetersPerSecond;

        // angle -= 180.0;

        if (Math.abs(getEncoderAngle() - angle) > 90.0) {
            if (angle > 0) {
                angle -= 180.0;
            } else {
                angle += 180.0;
            }

            speed = -speed;
        }

        setVelocityMeters(speed);
        setTurnAngle(angle);
    }

    public double getWheelPositionMeters() {
        double position = driveMotor.getPosition().getValue();
        // double position = driveMotor.getSelectedSensorPosition();
        position = (position * wheelDiameter * Math.PI) / (wheelGearRatio);
        // position = (position * wheelDiameter * Math.PI) / (wheelGearRatio * 2048);

        return position;
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getWheelPositionMeters(), Rotation2d.fromDegrees(getEncoderAngle()));
    }

}
