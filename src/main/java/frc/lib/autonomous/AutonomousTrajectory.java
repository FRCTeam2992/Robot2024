package frc.lib.autonomous;

import java.util.List;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

public enum AutonomousTrajectory {
    LoadStationMobility(PathPlanner.loadPathGroup("LoadStationMobility", new PathConstraints(1.5, 2.5))),
    WallMobility(PathPlanner.loadPathGroup("WallMobility", new PathConstraints(1.5, 2.5))),
    LoadStation2Scores(PathPlanner.loadPathGroup("LoadStation2Scores", 2.0, 1.5)),
    Wall3ScoresPart1(PathPlanner.loadPathGroup("Wall3Scores-Part1",
            new PathConstraints(4.5, 2.95),
            new PathConstraints(3.0, 2.25),
            new PathConstraints(4.5, 2.95),
            new PathConstraints(3.3, 2.25),
            new PathConstraints(4.5, 2.95))),
    Wall3ScoresPart2(PathPlanner.loadPathGroup("Wall3Scores-Part2",
            new PathConstraints(4.5, 2.95),
            new PathConstraints(3.3, 2.25),
            new PathConstraints(4.5, 2.95))),
    Wall2Scores(PathPlanner.loadPathGroup("Wall2Scores", 1.9, 3.0)),
    CenterIntakeBalanceWallSide(PathPlanner.loadPathGroup("CenterIntakeBalanceWallSide",
            new PathConstraints(1.6, 1.5),
            new PathConstraints(2.5, 2.5),
            new PathConstraints(2.0, 1.5),
            new PathConstraints(2.0, 1.5)));

    public List<PathPlannerTrajectory> trajectoryGroup;

    private AutonomousTrajectory(List<PathPlannerTrajectory> trajectoryGroup) {
        this.trajectoryGroup = trajectoryGroup;
    }
}
