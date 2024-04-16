// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import frc.lib.drive.swerve.OdometryThread;
import frc.lib.range.NoteDataPoint;
import frc.lib.range.NoteInterpolator;
import frc.robot.MyRobotState.RobotModeState;
import frc.robot.commands.AutoAim;
import frc.robot.commands.AutoIntake;
import frc.robot.commands.AutoMoveForwardBack;
import frc.robot.commands.AutoResetNote;
import frc.robot.commands.AutoRotateBot;
import frc.robot.commands.AutoRotateToHeading;
import frc.robot.commands.AutoShoot;
import frc.robot.commands.AutonomousLimitedAim;
import frc.robot.commands.DriveSticks;
import frc.robot.commands.ElevatorSticks;
import frc.robot.commands.HoldElevatorPosition;
import frc.robot.commands.HoldShooterPivot;
import frc.robot.commands.MoveElevatorToTarget;
import frc.robot.commands.MoveFeeder;
import frc.robot.commands.MoveIntake;
import frc.robot.commands.MoveShooter;
import frc.robot.commands.MoveShooterPivot;
import frc.robot.commands.ResetGyro;
import frc.robot.commands.SetElevatorTargetPosition;
import frc.robot.commands.SetNoteLocation;
import frc.robot.commands.SetOnTarget;
import frc.robot.commands.SetPivotTargetAngle;
import frc.robot.commands.SetPivotToTargetAngle;
import frc.robot.commands.SetShooterSpeedTarget;
import frc.robot.commands.SetSwerveAngle;
import frc.robot.commands.StartShooter;
import frc.robot.commands.StopIntake;
import frc.robot.commands.StopShooter;
import frc.robot.commands.StopShooterPivot;
import frc.robot.commands.ZeroElevator;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterPivot;

public class RobotContainer {

  public MyRobotState mRobotState;
  public NoteInterpolator mNoteInterpolator;

  public OdometryThread mOdometryThread;

  public final Drivetrain mDrivetrain;
  public final Shooter mShooter;
  public final Intake mIntake;
  public final Feeder mFeeder;
  public final ShooterPivot mShooterPivot;
  public final Elevator mElevator;
  public final PowerDistribution mPDH;

  public final LEDs intakeLEDs;
  // public final LEDs modeLEDs;

  public CommandXboxController controller0;
  public CommandXboxController controller1;

  public SetOnTarget setOnTargetCommand;
  public SetNoteLocation setNoteLocationCommand;

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

    intakeLEDs = new LEDs(mRobotState);
    // modeLEDs = new LEDs(mRobotState);

    controller0 = new CommandXboxController(0);
    controller1 = new CommandXboxController(1);

    setOnTargetCommand = new SetOnTarget(mElevator, mShooter, mShooterPivot, mRobotState);
    setNoteLocationCommand = new SetNoteLocation(mFeeder, mIntake, mRobotState);

    configureBindings();
    configureSmartDashboard();

    registerAutoCommandNames();
    
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
          mDrivetrain.setDoFieldOrient(false);
        }));// Disable Field Orient
    controller0.leftBumper().onFalse(new InstantCommand(
        () -> {
          mDrivetrain.setDoFieldOrient(true);
        }));// Disable Field Orient

    controller0.rightBumper().onTrue(new InstantCommand(
        () -> {
          mDrivetrain.setInSlowMode(false);
        })); // Slow Mode
    controller0.rightBumper().onFalse(new InstantCommand(
        () -> {
          mDrivetrain.setInSlowMode(true);
        })); // Slow Mode

    // ABXY
    controller0.a().whileTrue(new AutoRotateBot(mDrivetrain, true));
    controller0.y().onTrue(new InstantCommand(() -> {mDrivetrain.setEndgameTargetAngle(180.0);}));
    controller0.y().whileTrue(new AutoRotateBot(mDrivetrain, true));
    controller0.x().onTrue(new InstantCommand(() -> {mDrivetrain.setEndgameTargetAngle(60.0);}));
    controller0.x().whileTrue(new AutoRotateBot(mDrivetrain, true));
    controller0.b().onTrue(new InstantCommand(() -> {mDrivetrain.setEndgameTargetAngle(-60.0);}));
    controller0.b().whileTrue(new AutoRotateBot(mDrivetrain, true));

    // Start/Back
    controller0.start().onTrue(new ResetGyro(mDrivetrain));

    SmartDashboard.putData(new SetSwerveAngle(mDrivetrain, 90.0, 90.0, 90.0, 90.0));

    // Controller 1 (B)

    // Triggers
    controller1.rightTrigger(0.3)
        .onTrue(new SetPivotTargetAngle(mShooterPivot, 5.0)
            .andThen(new SetPivotToTargetAngle(mShooterPivot).withTimeout(2.0)));
    controller1.rightTrigger(0.3).whileTrue(new MoveIntake(mIntake, Constants.Intake.Speeds.outakingPieceSpeed, false)
        .alongWith(new MoveFeeder(mFeeder, Constants.Feeder.Speeds.outakingPieceSpeed, false)));

    // Bumpers
    controller1.leftBumper().onTrue(new InstantCommand(() -> {
      mRobotState.setRobotMode(RobotModeState.Passing);
    }));
    // .alongWith(new SetLEDStripColor(modeLEDs, Constants.LEDs.Colors.passing)));
    controller1.rightBumper()
        .onTrue(new SetPivotTargetAngle(mShooterPivot, Constants.ShooterPivot.Positions.intakingPiece)
            .andThen(new SetPivotToTargetAngle(mShooterPivot).withTimeout(2.0)));
    controller1.rightBumper().onTrue(new AutoIntake(mFeeder, mIntake, mRobotState));
    controller1.rightBumper().onTrue(
        new SetElevatorTargetPosition(mElevator, Constants.Elevator.Positions.intakingPiece)
            .andThen(new MoveElevatorToTarget(mElevator)));
    controller1.rightBumper().onTrue(new SetOnTarget(mElevator, mShooter, mShooterPivot, mRobotState));
    // POV
    controller1.povUp().whileTrue(new MoveShooterPivot(mShooterPivot, 0.13));
    controller1.povDown().whileTrue(new MoveShooterPivot(mShooterPivot, -0.10));
    controller1.povLeft().onTrue( new AutoResetNote(mFeeder, mIntake));
    // controller1.povLeft().whileTrue(new MoveFeeder(mFeeder, 0.4, false));
    // controller1.povLeft().whileTrue(new MoveIntake(mIntake, 0.3));
    controller1.povRight().onTrue(new InstantCommand(() -> {mRobotState.setRobotMode(RobotModeState.Endgame); }));

    // ABXY
    controller1.a().onTrue(new InstantCommand(() -> {
      mRobotState.setRobotMode(RobotModeState.Speaker);
    }));
    // .alongWith(new SetLEDStripColor(modeLEDs, Constants.LEDs.Colors.speaker)));
    controller1.b().onTrue(new InstantCommand(() -> {
      mRobotState.setRobotMode(RobotModeState.Amp);
    }));
    // .alongWith(new SetLEDStripColor(modeLEDs, Constants.LEDs.Colors.amp)));
    // controller1.y().onTrue(new InstantCommand(() -> {
    //   mRobotState.setRobotMode(RobotModeState.Endgame);
    // }));
    controller1.y().onTrue(new InstantCommand(() -> { mRobotState.setRobotMode(RobotModeState.DefaultSpeaker); }));
    // controller1.x().whileTrue(new SetShooterSpeedTarget(mShooter, 500));
    controller1.x().onTrue(new SetShooterSpeedTarget(mShooter, 0.0));
    controller1.x().onTrue(new StartShooter(mShooter));

    controller1.axisGreaterThan(1, Constants.Elevator.Climb.joyStickDeadBand)
        .whileTrue(new ElevatorSticks(mElevator, mShooterPivot));
    controller1.axisLessThan(1, -Constants.Elevator.Climb.joyStickDeadBand)
        .whileTrue(new ElevatorSticks(mElevator, mShooterPivot));

    //middle buttos
    controller1.start().onTrue(new InstantCommand(() -> {
      mShooterPivot.zeroPivotEncoder();
    }).ignoringDisable(true));
    controller1.back().onTrue(new InstantCommand(() -> {mElevator.zeroElevatorEncoders();}));


  }

  private void configureShuffleBoard() {
    SmartDashboard.putData("Ele 5 inch", new SetElevatorTargetPosition(mElevator, 5.0));
    SmartDashboard.putData("Ele 15 inch", new SetElevatorTargetPosition(mElevator, 15.0));

    // SmartDashboard.putNumber("Set Elevator Target Position", 0.0);
    SmartDashboard.putData(new ZeroElevator(mElevator));

  }

  private void configureSmartDashboard() {
    SmartDashboard.putData("Override Mode", new InstantCommand(()-> {mRobotState.setRobotMode(RobotModeState.Override);}));
    // .alongWith(new SetLEDStripColor(modeLEDs, Constants.LEDs.Colors.override)));

    SmartDashboard.putData("Move Robot Foward", new AutoMoveForwardBack(mDrivetrain, true, 2.0));
    SmartDashboard.putData("turn robot to 90", new AutoRotateToHeading(mDrivetrain, 90));

    SmartDashboard.putNumber("Set Shooter RPM", 0.0);
    SmartDashboard.putNumber("Set Pivot angle", 0.0);
    SmartDashboard.putData("Move pivot to position", new SetPivotToTargetAngle(mShooterPivot));
    // SmartDashboard.putData("Start Shooter", new StartShooter(mShooter));

    SmartDashboard.putData("Intake Foward", new MoveIntake(mIntake, 0.40, false)); // 2.25
    SmartDashboard.putData("Intake Reverse", new MoveIntake(mIntake, -0.40, false));

    SmartDashboard.putData("Feeder Foward", new MoveFeeder(mFeeder, 0.45, false)); // 2:1
    SmartDashboard.putData("feedforward limited", new MoveFeeder(mFeeder, 0.45, true));
    SmartDashboard.putData("Feeder Reverse", new MoveFeeder(mFeeder, -0.45, false));
    SmartDashboard.putData("Feeder Shoot", new MoveFeeder(mFeeder, 0.5, false).alongWith(new MoveIntake(mIntake, 0.4, false)));

    SmartDashboard.putData("Pivot Foward", new MoveShooterPivot(mShooterPivot, 0.05));
    SmartDashboard.putData("Pivot Reverse", new MoveShooterPivot(mShooterPivot, -0.05));

    SmartDashboard.putData("Start Shooter", new StartShooter(mShooter));
    SmartDashboard.putData("StopShooter", new MoveShooter(mShooter, 0.0));

    SmartDashboard.putData("AutoIntake", new MoveIntake(mIntake, 0.35, false).alongWith(new MoveFeeder(mFeeder, 0.40, false)));
    SmartDashboard.putData("AutoIntakeStop", new MoveIntake(mIntake, 0, false).alongWith(new MoveFeeder(mFeeder, 0, false)));

    SmartDashboard.putData("Zero pivot encoder", new InstantCommand(() -> {
      mShooterPivot.zeroPivotEncoder();
    }).ignoringDisable(true));

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

  mNoteInterpolator.addDataPoint(new NoteDataPoint(38, 2800, 56.0, 0.0));
  mNoteInterpolator.addDataPoint(new NoteDataPoint(42, 2800, 53.0, 0.0));
  mNoteInterpolator.addDataPoint(new NoteDataPoint(50, 2800, 49.0, 0.0));
  mNoteInterpolator.addDataPoint(new NoteDataPoint(55, 3200, 47.0, 0.0));
  mNoteInterpolator.addDataPoint(new NoteDataPoint(66, 3200, 44.0, 0.0));
  mNoteInterpolator.addDataPoint(new NoteDataPoint(78, 3400, 42, 0.0));
  mNoteInterpolator.addDataPoint(new NoteDataPoint(89, 3500, 41.0, 1.0));
  mNoteInterpolator.addDataPoint(new NoteDataPoint(99, 3700, 37.5, 1.0));
  mNoteInterpolator.addDataPoint(new NoteDataPoint(112, 3800, 35.0, 1.5));
  mNoteInterpolator.addDataPoint(new NoteDataPoint(120, 3900, 33.25, 1.5));
  mNoteInterpolator.addDataPoint(new NoteDataPoint(134, 4100, 31.25, 1.8));
  mNoteInterpolator.addDataPoint(new NoteDataPoint(150, 4200, 30.0, 4.0));
  mNoteInterpolator.addDataPoint(new NoteDataPoint(163, 4300, 28.5, 4.0));
  mNoteInterpolator.addDataPoint(new NoteDataPoint(170, 4500, 27.8, 4.0));
  mNoteInterpolator.addDataPoint(new NoteDataPoint(177, 4500, 27.5, 4.0));
  mNoteInterpolator.addDataPoint(new NoteDataPoint(185, 4500, 27.5, 4.0));

  }

  public void resetAllSubsystemState() {

    mDrivetrain.resetSubsystemState();
    mElevator.resetSubsystemState();
    mFeeder.resetSubsystemState();
    mIntake.resetSubsystemState();
    mShooter.resetSubsystemState();
    mShooterPivot.resetSubsystemState();
  }

  private void registerAutoCommandNames() {
    NamedCommands.registerCommand("autoAim",
        new AutoAim(mElevator, mShooterPivot, mShooter, mRobotState, mNoteInterpolator, mDrivetrain));
    NamedCommands.registerCommand("limitedAutoAim",
        new AutonomousLimitedAim(mElevator, mShooterPivot, mShooter, mRobotState, mNoteInterpolator, mDrivetrain));
    NamedCommands.registerCommand("pivot1stShot", new SetPivotTargetAngle(mShooterPivot, 56));
    NamedCommands.registerCommand("autoShoot",
        new AutoShoot(mIntake, mFeeder, mRobotState, mElevator, mShooterPivot, mShooter, 0).withTimeout(0.4));
    NamedCommands.registerCommand("autoIntake",
        new AutoIntake(mFeeder, mIntake, mRobotState).andThen(new WaitCommand(10.0)));
    NamedCommands.registerCommand("stopShooter", new StopShooter(mShooter).withTimeout(0.1));
    NamedCommands.registerCommand("stopIntake", new StopIntake(mIntake).withTimeout(0.1));
    NamedCommands.registerCommand("stopFeeder", new MoveFeeder(mFeeder, 0, false));
    NamedCommands.registerCommand("stopPivot", new StopShooterPivot(mShooterPivot).withTimeout(0.1));
    NamedCommands.registerCommand("driveStop", new AutoMoveForwardBack(mDrivetrain, true, 0).withTimeout(0.35));
    NamedCommands.registerCommand("startIntake", new MoveIntake(mIntake, 0.45, false));
    NamedCommands.registerCommand("startFeeder", new MoveFeeder(mFeeder, 1.0, false));
    NamedCommands.registerCommand("setPassingMode", new SequentialCommandGroup( 
      new SetPivotTargetAngle(mShooterPivot, 50.0), 
      new SetShooterSpeedTarget(mShooter, 1200.0),
      new SetPivotToTargetAngle(mShooterPivot),
      new StartShooter(mShooter)));
    NamedCommands.registerCommand("rotateAmpStart", new AutoRotateToHeading(mDrivetrain, -40.5));
    NamedCommands.registerCommand("rotateSourceStart", new AutoRotateToHeading(mDrivetrain, 43.0));
    NamedCommands.registerCommand("turnOnLimelightOdometry", new InstantCommand(() -> mDrivetrain.setLimeLightOdometryUpdates(true)));
    NamedCommands.registerCommand("turnOffLimelightOdometry", new InstantCommand(() -> mDrivetrain.setLimeLightOdometryUpdates(false)));
    NamedCommands.registerCommand("setSourceStartAimTarget", new SequentialCommandGroup( 
      new SetPivotTargetAngle(mShooterPivot, 43.3), 
      new SetShooterSpeedTarget(mShooter, 3280.0),
      new SetPivotToTargetAngle(mShooterPivot),
      new StartShooter(mShooter)));
  }
}
