// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import frc.lib.drive.swerve.OdometryThread;
import frc.lib.range.NoteDataPoint;
import frc.lib.range.NoteInterpolator;
import frc.robot.MyRobotState.RobotModeState;
import frc.robot.commands.AutoAim;
import frc.robot.commands.AutoIntake;
import frc.robot.commands.AutoRotateBot;
import frc.robot.commands.AutoShoot;
import frc.robot.commands.DisableFieldOrientToggle;
import frc.robot.commands.DriveSticks;
import frc.robot.commands.ElevatorSticks;
import frc.robot.commands.HoldElevatorPosition;
import frc.robot.commands.HoldShooterPivot;
import frc.robot.commands.MoveElevator;
import frc.robot.commands.MoveElevatorToTarget;
import frc.robot.commands.MoveFeeder;
import frc.robot.commands.MoveIntake;
import frc.robot.commands.MoveShooter;
import frc.robot.commands.MoveShooterPivot;
import frc.robot.commands.ResetGyro;
import frc.robot.commands.SetElevatorTargetPosition;
import frc.robot.commands.SetPivotToTargetAngle;
import frc.robot.commands.SetSwerveAngle;
import frc.robot.commands.StartShooter;
import frc.robot.commands.StopElevator;
import frc.robot.commands.StopIntake;
import frc.robot.commands.StopShooter;
import frc.robot.commands.StopShooterPivot;
import frc.robot.commands.ZeroElevator;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterPivot;

public class RobotContainer {

  public MyRobotState mRobotState;
  public NoteInterpolator mNoteInterpolator;

  public OdometryThread mOdometryThread;

  public Drivetrain mDrivetrain;
  public final Shooter mShooter;
  private Intake mIntake;
  private Feeder mFeeder;
  private ShooterPivot mShooterPivot;
  public final Elevator mElevator;
  public final PowerDistribution mPDH;

  public CommandXboxController controller0;
  public CommandXboxController controller1;

  private final SendableChooser<Command> autoChooser;

  public RobotContainer() {

    mRobotState = new MyRobotState();

    initNoteInterp();

    mDrivetrain = new Drivetrain();
    if (Constants.DrivetrainConstants.odometryThread) {
      mOdometryThread = new OdometryThread(mDrivetrain);
      mOdometryThread.setFastMode();
      mOdometryThread.start();
    }
    mDrivetrain.setDefaultCommand(new DriveSticks(mDrivetrain, mRobotState));

    mShooter = new Shooter();
    // mShooter.setDefaultCommand(new StopShooter(mShooter));

    mIntake = new Intake();
    mIntake.setDefaultCommand(new StopIntake(mIntake));

    mFeeder = new Feeder();
    mFeeder.setDefaultCommand(new MoveFeeder(mFeeder, 0.0, false));

    mShooterPivot = new ShooterPivot(mRobotState);
    mShooterPivot.setDefaultCommand(new HoldShooterPivot(mShooterPivot, mRobotState));

    mElevator = new Elevator(mRobotState);
    mElevator.setDefaultCommand(new HoldElevatorPosition(mElevator));

    mPDH = new PowerDistribution(1, PowerDistribution.ModuleType.kRev);

    controller0 = new CommandXboxController(0);
    controller1 = new CommandXboxController(1);

    configureBindings();
    configureSmartDashboard();

    autoChooser = AutoBuilder.buildAutoChooser("NothingCenter");
    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  private void configureBindings() {
    // Controller 0 (A)

    // Triggers
    controller0.leftTrigger(0.3).whileTrue(new AutoAim(mElevator, mShooterPivot,
        mShooter, mRobotState, mNoteInterpolator, mDrivetrain));
    // controller0.leftTrigger(0.3).whileTrue(new AutoRotateBot(mDrivetrain, true));
    controller0.rightTrigger(0.3)
        .whileTrue(new AutoShoot(mIntake, mFeeder, mRobotState, mElevator, mShooterPivot,
            mShooter, 1.0));
    // Bumpers
    controller0.leftBumper().onTrue(new InstantCommand(
        () -> {
          // mDrivetrain.setDoFieldOreint(false);
        }));// Disable Field Orient
    controller0.leftBumper().onFalse(new InstantCommand(
        () -> {
          // mDrivetrain.setDoFieldOreint(true);
        }));// Disable Field Orient

    controller0.rightBumper().onTrue(new InstantCommand(
        () -> {
          // mDrivetrain.setInSlowMode(true);
        })); // Slow Mode
    controller0.rightBumper().onFalse(new InstantCommand(
        () -> {
          // mDrivetrain.setInSlowMode(false);
        })); // Slow Mode

    // ABXY
    controller0.a().whileTrue(new AutoRotateBot(mDrivetrain, true));
    // controller0.x().whileTrue(new XWheels());
    controller0.y().whileTrue(new MoveIntake(mIntake, Constants.Intake.Speeds.outakingPieceSpeed)
        .alongWith(new MoveFeeder(mFeeder, Constants.Feeder.Speeds.outakingPieceSpeed, false)));

    // Start/Back
    controller0.start().onTrue(new ResetGyro(mDrivetrain));

    SmartDashboard.putData(new SetSwerveAngle(mDrivetrain, 90.0, 90.0, 90.0, 90.0));

    // Controller 1 (B)

    // Triggers
    controller1.rightTrigger(0.3).whileTrue(new MoveIntake(mIntake, Constants.Intake.Speeds.outakingPieceSpeed)
        .alongWith(new MoveFeeder(mFeeder, Constants.Feeder.Speeds.outakingPieceSpeed, false)));

    // Bumpers
    controller1.leftBumper().onTrue(new InstantCommand(() -> {
      mRobotState.setRobotMode(RobotModeState.DefaultSpeaker);
    }));
    controller1.rightBumper().onTrue(new AutoIntake(mFeeder, mIntake, mShooterPivot));
    controller1.rightBumper().onTrue(
        new SetElevatorTargetPosition(mElevator, Constants.Elevator.Positions.intakingPiece)
            .andThen(new MoveElevatorToTarget(mElevator)));

    // POV
    controller1.povUp().whileTrue(new MoveShooterPivot(mShooterPivot, 0.1));
    controller1.povDown().whileTrue(new MoveShooterPivot(mShooterPivot, -0.04));

    // ABXY
    controller1.a().onTrue(new InstantCommand(() -> {
      mRobotState.setRobotMode(RobotModeState.Speaker);
    }));
    controller1.b().onTrue(new InstantCommand(() -> {
      mRobotState.setRobotMode(RobotModeState.Amp);
    }));
    controller1.y().onTrue(new InstantCommand(() -> {
      mRobotState.setRobotMode(RobotModeState.Endgame);
    }));

    controller0.povUp().whileTrue(new MoveElevator(mElevator, mShooterPivot, 0.15));
    controller0.povDown().whileTrue(new MoveElevator(mElevator, mShooterPivot, -0.05));

    controller1.axisGreaterThan(1, Constants.Elevator.Climb.joyStickDeadBand)
        .whileTrue(new ElevatorSticks(mElevator, mShooterPivot));
    controller1.axisLessThan(1, -Constants.Elevator.Climb.joyStickDeadBand)
        .whileTrue(new ElevatorSticks(mElevator, mShooterPivot));

  }

  private void configureShuffleBoard() {
    SmartDashboard.putData("Ele 5 inch", new SetElevatorTargetPosition(mElevator, 5.0));
    SmartDashboard.putData("Ele 15 inch", new SetElevatorTargetPosition(mElevator, 15.0));

    // SmartDashboard.putNumber("Set Elevator Target Position", 0.0);
    SmartDashboard.putData(new ZeroElevator(mElevator));

  }

  private void configureSmartDashboard() {
    SmartDashboard.putNumber("Set Shooter RPM", 0.0);
    SmartDashboard.putNumber("Set Pivot angle", 0.0);
    SmartDashboard.putData("Move pivot to position", new SetPivotToTargetAngle(mShooterPivot));
    // SmartDashboard.putData("Start Shooter", new StartShooter(mShooter));

    SmartDashboard.putData("Intake Foward", new MoveIntake(mIntake, 0.40)); // 2.25
    SmartDashboard.putData("Intake Reverse", new MoveIntake(mIntake, -0.40));

    SmartDashboard.putData("Feeder Foward", new MoveFeeder(mFeeder, 0.45, false)); // 2:1
    SmartDashboard.putData("feedforward limited", new MoveFeeder(mFeeder, 0.45, true));
    SmartDashboard.putData("Feeder Reverse", new MoveFeeder(mFeeder, -0.45, false));
    SmartDashboard.putData("Feeder Shoot", new MoveFeeder(mFeeder, 1.0, false).alongWith(new MoveIntake(mIntake, 0.4)));

    SmartDashboard.putData("Pivot Foward", new MoveShooterPivot(mShooterPivot, 0.05));
    SmartDashboard.putData("Pivot Reverse", new MoveShooterPivot(mShooterPivot, -0.05));

    SmartDashboard.putData("Start Shooter", new StartShooter(mShooter));
    SmartDashboard.putData("StopShooter", new MoveShooter(mShooter, 0.0));

    SmartDashboard.putData("AutoIntake", new MoveIntake(mIntake, 0.15).alongWith(new MoveFeeder(mFeeder, 0.20, false)));
    SmartDashboard.putData("AutoIntakeStop", new MoveIntake(mIntake, 0).alongWith(new MoveFeeder(mFeeder, 0, false)));

    SmartDashboard.putData("Zero pivot encoder", new InstantCommand(() -> {
      mShooterPivot.zeroPivotEncoder();
    }));

    SmartDashboard.putNumber("Set Elevator Position", 0.0);
    SmartDashboard.putData("Move Elevator", new MoveElevatorToTarget(mElevator));
    SmartDashboard.putData("zero ele target", new SetElevatorTargetPosition(mElevator, 0.0));
    SmartDashboard.putData("zero ele encoder", new ZeroElevator(mElevator));

    SmartDashboard.putData("Reset odometry", mDrivetrain.ResetOdometry());
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  public CommandXboxController getController0() {
    return this.controller0;
  }

  private void initNoteInterp() {
    mNoteInterpolator = new NoteInterpolator();

    mNoteInterpolator.addDataPoint(new NoteDataPoint(42, 3300, 49));
    mNoteInterpolator.addDataPoint(new NoteDataPoint(53, 3500, 42));
    mNoteInterpolator.addDataPoint(new NoteDataPoint(64, 4200, 38.0));
    mNoteInterpolator.addDataPoint(new NoteDataPoint(75, 4200, 36.0));
    mNoteInterpolator.addDataPoint(new NoteDataPoint(86, 4200, 33.0));
    mNoteInterpolator.addDataPoint(new NoteDataPoint(109, 4200, 28.5));
    mNoteInterpolator.addDataPoint(new NoteDataPoint(134, 3250, 28.5));

  }
}
