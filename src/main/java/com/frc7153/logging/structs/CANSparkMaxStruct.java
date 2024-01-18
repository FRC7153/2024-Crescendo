package com.frc7153.logging.structs;

import java.nio.ByteBuffer;

import org.ejml.simple.UnsupportedOperation;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.util.struct.Struct;

public class CANSparkMaxStruct implements Struct<CANSparkMax> {
    @Override
    public Class<CANSparkMax> getTypeClass() {
        return CANSparkMax.class;
    }

    @Override
    public String getTypeString() {
        return "struct:CANSparkMax";
    }

    @Override
    public int getSize() {
        return (kSizeDouble * 5);
    }

    @Override
    public String getSchema() {
        return "double position;double velocity;double motorTemp;double outputCurrent;double outputDutyCycle";
    }

    @Override
    public CANSparkMax unpack(ByteBuffer bb) {
        throw new UnsupportedOperation("CANSparkMaxes can't be unpacked from a byte buffer!");
    }

    @Override
    public void pack(ByteBuffer bb, CANSparkMax value) {
        RelativeEncoder enc = value.getEncoder();

        bb.putDouble(enc.getPosition());
        bb.putDouble(enc.getVelocity());
        bb.putDouble(value.getMotorTemperature());
        bb.putDouble(value.getOutputCurrent());
        bb.putDouble(value.getAppliedOutput());
    }
    
    /**
     * Static instance of this.
     */
    public static CANSparkMaxStruct struct = new CANSparkMaxStruct();
}
