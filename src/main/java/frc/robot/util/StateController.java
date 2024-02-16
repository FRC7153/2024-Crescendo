package frc.robot.util;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringEntry;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * Handles the robot's state
 */
public class StateController {
    /** The robot's NOTE state */
    public static enum NoteState {
        /** There is no NOTE in the robot */
        EMPTY,
        /** The robot is intaking a NOTE */
        PROCESSING,
        /** The robot is carrying a NOTE */
        LOADED
    }

    // Current state
    private static NoteState state;
    private static Trigger robotLoadedTrigger;
    private static StringPublisher noteStatePublisher = NetworkTableInstance.getDefault().getTable("State").getStringTopic("Note State").publish();

    // Logging
    private static StringLogEntry noteStateLog = new StringLogEntry(DataLogManager.getLog(), "Robot/NoteState");

    static {
        // Initialize
        setState(NoteState.EMPTY);
        robotLoadedTrigger = new Trigger(() -> getState() == NoteState.LOADED);
    }

    /**
     * Sets the robot's note state
     * @param state
     */
    public static void setState(NoteState state) {
        StateController.state = state;
        noteStateLog.append(state.name());
        noteStatePublisher.set(state.name());
    }

    /**
     * Gets the robot's note state
     * @return
     */
    public static NoteState getState() { return state; }

    /**
     * Gets a trigger for if the robot is LOADED
     * @return
     */
    public static Trigger getLoadedTrigger() { return robotLoadedTrigger; }
}
