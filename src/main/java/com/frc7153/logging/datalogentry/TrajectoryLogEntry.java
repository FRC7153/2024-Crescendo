package com.frc7153.logging.datalogentry;

import java.util.List;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.StructArrayLogEntry;

/**
 * For putting PathPlannerLib's trajectories or WPI's trajectories into log files.
 */
public class TrajectoryLogEntry {
    private StructArrayLogEntry<Pose2d> entry;

    public TrajectoryLogEntry(DataLog log, String name, String metadata, long timestamp) {
        entry = StructArrayLogEntry.create(log, name, Pose2d.struct, metadata, timestamp);
    }

    public TrajectoryLogEntry(DataLog log, String name) {
        this(log, name, "", 0);
    }

    /**
     * Appends a trajectory to the log. This probably should not be run periodically,
     * only when necessary (new trajectories are loaded).
     * @param trajectory Trajectory to log
     * @param timestamp Time stamp (0 to indicate now)
     */
    public void append(PathPlannerPath trajectory, long timestamp) {
        List<PathPoint> pts = trajectory.getAllPathPoints();
        Pose2d[] poses = new Pose2d[pts.size()];

        // Get Pose2d objects
        for (int x = 0; x < pts.size(); x++) {
            poses[x] = new Pose2d(pts.get(x).position, pts.get(x).rotationTarget.getTarget());
        }

        entry.append(poses, timestamp);
    }

    /**
     * Appends a trajectory to the log. This probably should not be run periodically,
     * only when necessary (new trajectories are loaded).
     * @param trajectory Trajectory to log
     */
    public void append(PathPlannerPath trajectory) {
        append(trajectory, 0);
    }

    /**
     * Appends a trajectory to the log. This probably should not be run periodically,
     * only when necessary (new trajectories are loaded).
     * @param trajectory Trajectory to log
     * @param timestamp Time stamp (0 to indicate now)
     */
    public void append(Trajectory trajectory, long timestamp) {
        List<Trajectory.State> states = trajectory.getStates();
        Pose2d[] poses = new Pose2d[states.size()];

        // Get Pose2d objects
        for (int x = 0; x < states.size(); x++) {
            poses[x] = states.get(x).poseMeters;
        }

        entry.append(poses, timestamp);
    }

    /**
     * Appends a trajectory to the log. This probably should not be run periodically,
     * only when necessary (new trajectories are loaded).
     * @param trajectory Trajectory to log
     */
    public void append(Trajectory trajectory) {
        append(trajectory, 0);
    }
}
