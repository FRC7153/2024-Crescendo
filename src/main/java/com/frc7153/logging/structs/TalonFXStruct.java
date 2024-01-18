package com.frc7153.logging.structs;

import java.nio.ByteBuffer;

import org.ejml.simple.UnsupportedOperation;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.util.struct.Struct;

public class TalonFXStruct implements Struct<TalonFX> {
    @Override
    public Class<TalonFX> getTypeClass() {
        return TalonFX.class;
    }

    @Override
    public String getTypeString() {
        return "struct:TalonFX";
    }

    @Override
    public int getSize() {
        return (kSizeDouble * 5) + kSizeInt32;
    }

    @Override
    public String getSchema() {
        return "double position;double velocity;double acceleration;int32 controlMode;double deviceTemp;double procTemp";
    }

    @Override
    public TalonFX unpack(ByteBuffer bb) {
        throw new UnsupportedOperation("TalonFXs can't be unpacked from a byte buffer!");
    }

    @Override
    public void pack(ByteBuffer bb, TalonFX value) {
        bb.putDouble(value.getPosition().getValue());
        bb.putDouble(value.getVelocity().getValue());
        bb.putDouble(value.getAcceleration().getValue());
        bb.putInt(value.getControlMode().getValue().value);
        bb.putDouble(value.getDeviceTemp().getValue());
        bb.putDouble(value.getProcessorTemp().getValue());
        value.getControlMode().getValue();
    }
    
    /**
     * Static instance of this.
     */
    public static TalonFXStruct struct = new TalonFXStruct();
}
