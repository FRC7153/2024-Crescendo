package frc.robot.util;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
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
        /** There is no NOTE in the robot (default) */
        EMPTY,
        /** The robot is carrying a NOTE */
        LOADED
    }

    /** The robot's OBJECTIVE state */
    public static enum ObjectiveState {
        /** Robot is scoring NOTES only */
        SCORING,
        /** Robot is DEFENDING only (default) */
        DEFENDING,
        /** Robot is CLIMBING only */
        CLIMBING
    }

    // Current state
    private static NoteState noteState;
    private static StringPublisher noteStatePublisher;

    private static ObjectiveState objectiveState;
    private static StringPublisher objectiveStatePublisher;

    // Logging
    private static StringLogEntry noteStateLog = new StringLogEntry(DataLogManager.getLog(), "Robot/NoteState");
    private static StringLogEntry objectiveStateLog = new StringLogEntry(DataLogManager.getLog(), "Robot/ObjectiveState");

    static {
        // Initialize NT
        NetworkTable stateTable = NetworkTableInstance.getDefault().getTable("State");
        noteStatePublisher = stateTable.getStringTopic("Note State").publish();
        objectiveStatePublisher = stateTable.getStringTopic("Objective State").publish();

        // Initialize
        setNoteState(NoteState.EMPTY);
        setObjectiveState(ObjectiveState.DEFENDING);
    }

    /**
     * Sets the robot's note state
     * @param state
     */
    public static void setNoteState(NoteState state) {
        StateController.noteState = state;
        noteStateLog.append(state.name());
        noteStatePublisher.set(state.name());
    }

    /**
     * Gets the robot's note state
     * @return
     */
    public static NoteState getNoteState() { return noteState; }

    /**
     * Sets the robot's objective state
     * @param state
     */
    public static void setObjectiveState(ObjectiveState state) {
        StateController.objectiveState = state;
        objectiveStateLog.append(state.name());
        objectiveStatePublisher.set(state.name());
    }

    /**
     * Gets the robot's objective state
     */
    public static ObjectiveState getObjectiveState() { return objectiveState; }

    /**
     * Creates a trigger with the states. Leave {@code null} if it's not required.
     * @param targetNoteState
     * @param targetObjectiveState
     * @return
     */
    public static Trigger buildTrigger(NoteState targetNoteState, ObjectiveState targetObjectiveState) {
        return new Trigger(() -> {
            return (targetNoteState == null || targetNoteState.equals(getNoteState())) &&
                (targetObjectiveState == null || targetObjectiveState.equals(getObjectiveState()));
        });
    }
}
