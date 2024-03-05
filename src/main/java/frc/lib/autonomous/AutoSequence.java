package frc.lib.autonomous;

import java.util.Arrays;
import java.util.List;

public enum AutoSequence {
    Do_Nothing("Do Nothing",
            AutoStartPosition.LoadStationMidCube,
            AutoStartPosition.CenterMidCube,
            AutoStartPosition.WallMidCube),
    SideMobilityOnly("Side Mobility Only",
            AutoStartPosition.LoadStationMidCube,
            AutoStartPosition.WallMidCube),
    Side2Scores("Side 2 Scores",
            AutoStartPosition.LoadStationMidCube,
            AutoStartPosition.WallMidCube),
    Side3Scores("Side 3 Scores"),
        //     AutoStartPosition.LoadStationMidCube,
        //     AutoStartPosition.WallMidCube),
    CenterIntakeBalance("Center Intake Balance",
            AutoStartPosition.CenterMidCube);

    public String description;
    public List<AutoStartPosition> allowedStartPositions;

    private AutoSequence(String description, AutoStartPosition... allowedStartPositions) {
        this.description = description;
        this.allowedStartPositions = Arrays.asList(allowedStartPositions);
    }
}