package com.frc7153.diagnostics;

/**
 * Generic Device that can be checked
 */
public abstract class CheckableDevice {
    /**
     * Use {@code getOK()} instead, to update {@code hasChanged}
     * @return If the device is currently OK
     */
    protected abstract boolean isOk();

    /**
     * @return Current device message. This will only be called IF there is
     * an error
     */
    public abstract String getMessage();

    /**
     * @return ID for this device, used for debugging
     */
    public abstract String getID();

    // Previous OK value
    private Boolean prev;
    private boolean hasChanged = true;

    /**
     * Get OK value, and save previous
     * @return OK value
     */
    public final boolean getOK() {
        // Check first run
        if (prev == null) {
            prev = isOk();
            hasChanged = true;
        } else {
            boolean ok = isOk();
            hasChanged = (prev != ok);
            prev = ok;
        }

        return prev;
    }

    /**
     * @return If the OK value has changed. On first run, this is always true
     */
    public final boolean hasChanged() { return hasChanged; }
}
