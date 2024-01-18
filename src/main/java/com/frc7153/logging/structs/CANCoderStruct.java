package com.frc7153.logging.structs;

import java.nio.ByteBuffer;

import org.ejml.simple.UnsupportedOperation;

import com.ctre.phoenix6.hardware.CANcoder;

import edu.wpi.first.util.struct.Struct;

public class CANCoderStruct implements Struct<CANcoder> {
    @Override
    public Class<CANcoder> getTypeClass() {
        return CANcoder.class;
    }

    @Override
    public String getTypeString() {
        return "struct:CANcoder";
    }

    @Override
    public int getSize() {
        return (kSizeDouble * 3);
    }

    @Override
    public String getSchema() {
        return "double position;double velocity;double supplyVoltage";
    }

    @Override
    public CANcoder unpack(ByteBuffer bb) {
        throw new UnsupportedOperation("CANcoders can't be unpacked from a byte buffer!");
    }

    @Override
    public void pack(ByteBuffer bb, CANcoder value) {
        bb.putDouble(value.getPosition().getValue());
        bb.putDouble(value.getVelocity().getValue());
        bb.putDouble(value.getSupplyVoltage().getValue());
    }
    
    /**
     * Static instance of this.
     */
    public static CANCoderStruct struct = new CANCoderStruct();
}
