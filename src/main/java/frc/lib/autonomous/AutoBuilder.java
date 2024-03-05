// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.autonomous;

import java.util.HashMap;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.FollowPathWithEvents;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.lib.manipulator.Waypoint;
import frc.lib.manipulator.Waypoint.OuttakeType;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.RobotState.GridTargetingPosition;
import frc.robot.RobotState.IntakeModeState;
import frc.robot.commands.BalanceRobotPID;
import frc.robot.commands.ClawOuttake;
import frc.robot.commands.IntakeGamePiece;
import frc.robot.commands.MoveArmToPoint;
import frc.robot.commands.MoveClaw;
import frc.robot.commands.SetArmPosition;
import frc.robot.commands.SetLimeLightOdometryUpdates;
import frc.robot.commands.StopClaw;
import frc.robot.commands.groups.FollowTrajectoryCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.LEDs;

/** Add your docs here. */
public class AutoBuilder {
    private RobotState mRobotState;
    private Drivetrain mDrivetrain;
    private Arm mArm;
    private Claw mClaw;
    private LEDs mLEDs;

    private SendableChooser<AutoStartPosition> autoStartChooser;
    private SendableChooser<AutoSequence> autoSequenceChooser;
    private SendableChooser<AutoPreloadScore> autoPreloadScoreChooser;

    private HashMap<String, Command> eventMap = new HashMap<>();

    public AutoBuilder(RobotState robotState, Drivetrain drivetrain, Arm arm, Claw claw, LEDs leds) {
        mRobotState = robotState;
        mDrivetrain = drivetrain;
        mArm = arm;
        mClaw = claw;
        mLEDs = leds;

        eventMap.put("SetIntakeModeCube", new InstantCommand(
            () -> mRobotState.intakeMode = IntakeModeState.Cube
        ));
        eventMap.put("ArmMoveLowFront", new ScheduleCommand(
            new SetArmPosition(mArm, GridTargetingPosition.LowFront.towerWaypoint.angle())
        ).asProxy());
        eventMap.put("TowerMoveMidFront", new ScheduleCommand(
            new SetArmPosition(mArm, GridTargetingPosition.MidFront.towerWaypoint.angle())
        ).asProxy());
        eventMap.put("TowerMoveGroundIntake", new ScheduleCommand(
            new SetArmPosition(mArm, Constants.TowerConstants.cubeGroundIntake.angle())
        ).asProxy());
        eventMap.put("TowerMoveStowed", new ScheduleCommand(
            new SetArmPosition(mArm, Constants.TowerConstants.normal.angle())
        ).asProxy());
        eventMap.put("TowerMoveLoadStation", new ScheduleCommand(
            new SetArmPosition(mArm, Constants.TowerConstants.singleLoadStation.angle())
        ).asProxy());
        eventMap.put("StartCubeIntake", new IntakeGamePiece(mClaw, mLEDs, mRobotState).asProxy());
        eventMap.put("StartCubeOuttake", new ClawOuttake(mClaw, mRobotState).asProxy());
        eventMap.put("StopClaw", new StopClaw(mClaw).asProxy());
        eventMap.put("EndIntake", new MoveClaw(mClaw, 0.2).asProxy());
    }

    public void setupAutoSelector() {
        // Setup choosers for start position
        autoStartChooser = new SendableChooser<>();
        autoStartChooser.addOption(AutoStartPosition.LoadStationMidCube.description,
                AutoStartPosition.LoadStationMidCube);
        autoStartChooser.addOption(AutoStartPosition.WallMidCube.description, AutoStartPosition.WallMidCube);
        autoStartChooser.setDefaultOption(AutoStartPosition.CenterMidCube.description, AutoStartPosition.CenterMidCube);

        SmartDashboard.putData("Auto Start Position", autoStartChooser);

        // Setup chooser for preload scoring
        autoPreloadScoreChooser = new SendableChooser<>();
        autoPreloadScoreChooser.setDefaultOption(AutoPreloadScore.No_Preload.description, AutoPreloadScore.No_Preload);
        // autoPreloadScoreChooser.addOption(AutoPreloadScore.Mid_Cube.description,
        // AutoPreloadScore.Mid_Cube);
        autoPreloadScoreChooser.addOption(AutoPreloadScore.Mid_Cube.description,
                AutoPreloadScore.Mid_Cube);

        SmartDashboard.putData("Preload Score?", autoPreloadScoreChooser);

        // Setup chooser for auto sequence
        autoSequenceChooser = new SendableChooser<>();
        autoSequenceChooser.setDefaultOption(AutoSequence.Do_Nothing.description, AutoSequence.Do_Nothing);
        autoSequenceChooser.addOption(AutoSequence.SideMobilityOnly.description,
                AutoSequence.SideMobilityOnly);
        autoSequenceChooser.addOption(AutoSequence.Side2Scores.description, AutoSequence.Side2Scores);
        autoSequenceChooser.addOption(AutoSequence.Side3Scores.description, AutoSequence.Side3Scores);
        autoSequenceChooser.addOption(AutoSequence.CenterIntakeBalance.description, AutoSequence.CenterIntakeBalance);

        SmartDashboard.putData("Auto Sequence", autoSequenceChooser);

    }

    public AutoStartPosition getAutoStartPosition() {
        return autoStartChooser.getSelected();
    }

    public AutoPreloadScore getAutoPreloadScore() {
        return autoPreloadScoreChooser.getSelected();
    }

    public AutoSequence getAutoSequence() {
        return autoSequenceChooser.getSelected();
    }

    public boolean autoStartCompatible() {
        // Returns true if the Auto Start Position is valid for the current selected
        // sequence
        return autoSequenceChooser.getSelected().allowedStartPositions.contains(
                autoStartChooser.getSelected());
    }

    private Command setupAutoInitialScoreCommand() {
        Command initialScoreCommand;
        
        if (getAutoStartPosition() == null || getAutoStartPosition().getStartPose() == null) {
            return new InstantCommand();
        } else {
            Pose2d startingPose = getAutoStartPosition().getStartPose();
            initialScoreCommand = new InstantCommand(() -> mDrivetrain.resetOdometryToPose(startingPose));
            switch (getAutoPreloadScore()) {
                case Mid_Cube:
                    initialScoreCommand = initialScoreCommand
                            .andThen(new InstantCommand(() -> {
                                mRobotState.currentOuttakeType = OuttakeType.Mid_Cube;
                                mRobotState.intakeMode = IntakeModeState.Cube;
                            }))
                            .andThen(
                                    new SetArmPosition(mArm, GridTargetingPosition.MidFront.towerWaypoint.angle()).asProxy()
                                            .raceWith(new MoveClaw(mClaw, 0.2).asProxy()))
                            .andThen(new ClawOuttake(mClaw, mRobotState).asProxy().withTimeout(0.5));
                    break;
                case No_Preload:
                default:
            }
            return initialScoreCommand;
        }
    }

    private Command setupAutoPathFollowCommand(boolean isFirstPath) {
        Command followCommand = new InstantCommand();
        isFirstPath = true;
        switch (getAutoSequence()) {
            case Do_Nothing:
                break;
            case SideMobilityOnly:
                if (getAutoStartPosition() == AutoStartPosition.LoadStationMidCube) {
                    for (PathPlannerTrajectory path : AutonomousTrajectory.LoadStationMobility.trajectoryGroup) {
                        followCommand = followCommand.andThen(new FollowPathWithEvents(
                                new FollowTrajectoryCommand(mDrivetrain, path, isFirstPath),
                                path.getMarkers(),
                                eventMap));
                        isFirstPath = false; // Make sure it's false for subsequent paths
                    }
                } else if (getAutoStartPosition() == AutoStartPosition.WallMidCube) {
                    for (PathPlannerTrajectory path : AutonomousTrajectory.WallMobility.trajectoryGroup) {
                        followCommand = followCommand.andThen(new FollowPathWithEvents(
                                new FollowTrajectoryCommand(mDrivetrain, path, isFirstPath),
                                path.getMarkers(),
                                eventMap));
                        isFirstPath = false; // Make sure it's false for subsequent paths
                    }
                }
                break;
            case Side2Scores:
                if (getAutoStartPosition() == AutoStartPosition.LoadStationMidCube) {
                    for (PathPlannerTrajectory path : AutonomousTrajectory.LoadStation2Scores.trajectoryGroup) {
                        followCommand = followCommand.andThen(new FollowPathWithEvents(
                                new FollowTrajectoryCommand(mDrivetrain, path, isFirstPath),
                                path.getMarkers(),
                                eventMap));
                        isFirstPath = false; // Make sure it's false for subsequent paths
                    }
                } else if (getAutoStartPosition() == AutoStartPosition.WallMidCube) {
                    for (PathPlannerTrajectory path : AutonomousTrajectory.Wall2Scores.trajectoryGroup) {
                        followCommand = followCommand.andThen(new FollowPathWithEvents(
                                new FollowTrajectoryCommand(mDrivetrain, path, isFirstPath),
                                path.getMarkers(),
                                eventMap));
                        isFirstPath = false; // Make sure it's false for subsequent paths
                    }
                }
                followCommand = followCommand
                        .andThen(new InstantCommand(() -> {
                            mDrivetrain.stopDrive();
                            mRobotState.currentOuttakeType = OuttakeType.Front_Low_Cube;
                        }))
                        .andThen(new WaitCommand(0.1))
                        .andThen(new SetArmPosition(mArm, Constants.TowerConstants.frontCubeLow.angle()).asProxy())
                        .andThen(new ClawOuttake(mClaw, mRobotState).asProxy().withTimeout(1.0))
                        .andThen(new SetArmPosition(mArm, Constants.TowerConstants.normal.angle()).asProxy());
                break;
            case Side3Scores:
                // followCommand = new InstantCommand();
                // if (getAutoStartPosition() == AutoStartPosition.LoadStationMidCube) {
                // for (PathPlannerTrajectory path :
                // AutonomousTrajectory.LoadStation3ScoresPart1.trajectoryGroup) {
                // followCommand = followCommand.andThen(new FollowPathWithEvents(
                // new FollowTrajectoryCommand(mDrivetrain, path, isFirstPath),
                // path.getMarkers(),
                // eventMap));
                // isFirstPath = false; // Make sure it's false for subsequent paths
                // }
                // followCommand = followCommand.andThen(new WaitCommand(1.0))
                // .andThen(new ClawOuttake(mClaw, mRobotState).withTimeout(1.0))
                // .andThen(new SetArmPosition(mArm, Constants.TowerConstants.normal.angle()));
                // followCommand = followCommand.andThen(new FollowPathWithEvents(
                // new FollowTrajectoryCommand(mDrivetrain, path, isFirstPath),
                // path.getMarkers(),
                // eventMap));
                // }
                // } else if (getAutoStartPosition() == AutoStartPosition.WallMidCube) {
                // for (PathPlannerTrajectory path :
                // AutonomousTrajectory.Wall3ScoresPart1.trajectoryGroup) {
                // followCommand = followCommand.andThen(new FollowPathWithEvents(
                // new FollowTrajectoryCommand(mDrivetrain, path, isFirstPath),
                // path.getMarkers(),
                // eventMap));
                // isFirstPath = false; // Make sure it's false for subsequent paths
                // }
                // followCommand = followCommand.andThen(new InstantCommand(() -> {
                // mDrivetrain.stopDrive();
                // mRobotState.currentOuttakeType = OuttakeType.Rear_Low_Cube;
                // }))
                // .andThen(new MoveClaw(mClaw,
                // Waypoint.OuttakeType.Rear_Low_Cube.speed).withTimeout(0.4));
                // for (PathPlannerTrajectory path :
                // AutonomousTrajectory.Wall3ScoresPart2.trajectoryGroup) {
                // followCommand = followCommand.andThen(new FollowPathWithEvents(
                // new FollowTrajectoryCommand(mDrivetrain, path, isFirstPath),
                // path.getMarkers(),
                // eventMap));
                // }
                // }

                // followCommand = followCommand.andThen(new InstantCommand(() ->
                // mDrivetrain.stopDrive()))
                // .andThen(new MoveClaw(mClaw, Waypoint.OuttakeType.Rear_Low_Cube.speed));
                break;
            case CenterIntakeBalance:
                // followCommand = followCommand.andThen(new
                // SetLimeLightOdometryUpdates(mRobotState, mDrivetrain, false));
                System.out.println("Selected center balance path");
                if (getAutoStartPosition() == AutoStartPosition.CenterMidCube) {
                    System.out.println("Running center balance path");
                    for (PathPlannerTrajectory path : AutonomousTrajectory.CenterIntakeBalanceWallSide.trajectoryGroup) {
                        followCommand = followCommand.andThen(new FollowPathWithEvents(
                                new FollowTrajectoryCommand(mDrivetrain, path, isFirstPath),
                                path.getMarkers(),
                                eventMap));
                        isFirstPath = false; // Make sure it's false for subsequent paths
                    }
                }
                followCommand = followCommand.andThen(new BalanceRobotPID(mDrivetrain));
                break;
        }
        return followCommand;
    }

    public Command buildAutoCommand() {
        Command autoPathCommand = null;
        Command initialScoreCommand = null;
        Command afterInitialScoreCommand = null;

        // Setup the initial preload scoring path and command sequence
        initialScoreCommand = setupAutoInitialScoreCommand();

        if (!autoStartCompatible()) {
            // We have incompatible starting position for sequence.
            // Run only the initial score command, which in the case of
            // No_Preload, just resets odometry and stops.
            return initialScoreCommand;
        } else {
            // Starting position is compatible, so setup the path following command,
            // then build a parallel group to move from scoring position while driving
            autoPathCommand = setupAutoPathFollowCommand(true);
            afterInitialScoreCommand = autoPathCommand;
        }

        // If we've completed the above, we should always have a Command object for
        // both initialScoreCommand and afterInitialScoreCommand (either or both of
        // which may be just a dummy InstantCommand that does nothing), so we can now
        // return a sequence of those Commands.
        return initialScoreCommand.andThen(afterInitialScoreCommand);
    }

}
