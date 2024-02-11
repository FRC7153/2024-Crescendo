package com.frc7153.diagnostics.devices;

import com.frc7153.diagnostics.CheckableDevice;

import edu.wpi.first.networktables.Topic;

public class NTTopicDevice extends CheckableDevice {
    private Topic topic;

    // Constructors
    public NTTopicDevice(Topic topic) {
        this.topic = topic;
    }

    @Override
    protected boolean isOk() {
        return topic.exists();
    }

    @Override
    public String getMessage() {
        if (!topic.exists()) return "Topic does not exist!";
        return "Ok";
    }

    @Override
    public void performLogging() {} // None

    @Override
    public String getID() {
        return String.format("NT - %s", topic.getName());
    }
}
