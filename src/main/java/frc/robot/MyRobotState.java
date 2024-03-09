// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/** Add your docs here. */
public class MyRobotState {

    public static enum RobotModeState {
        Auto,
        Override,
        Speaker,
        Amp,
        Endgame,
        DefaultSpeaker,
        Passing
    }

    private RobotModeState robotMode = RobotModeState.Speaker;

    public MyRobotState() {
        robotMode = RobotModeState.Speaker;  // Always start as normal at boot
    }

    public void setRobotMode(RobotModeState mode) {
        robotMode = mode;
    }

    public RobotModeState getRobotMode() {
        return robotMode;
    }

    public boolean isAutoMode() {
        return robotMode == RobotModeState.Auto;
    }

    public boolean isAmpMode() {
        return robotMode == RobotModeState.Amp;
    }

    public boolean isEndgameMode() {
        return robotMode == RobotModeState.Endgame;
    }

    public boolean IsOverrideMode() {
        return robotMode == RobotModeState.Override;
    }

    public boolean isSpeakerMode() {
        return robotMode == RobotModeState.Speaker;
    }

    public boolean isDefaultSpeakerMode() {
        return robotMode == RobotModeState.DefaultSpeaker;
    }

    public boolean isPassing() {
        return robotMode == RobotModeState.Passing;
    }
    
}
