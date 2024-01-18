package com.frc7153.logging.structs;

import java.nio.ByteBuffer;

import org.ejml.simple.UnsupportedOperation;

import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.wpilibj.PowerDistribution;

public class PowerDistributionStruct implements Struct<PowerDistribution> {
    @Override
    public Class<PowerDistribution> getTypeClass() {
        return PowerDistribution.class;
    }

    @Override
    public String getTypeString() {
        return "struct:PowerDistribution";
    }

    @Override
    public int getSize() {
        return (kSizeDouble * 4);
    }

    @Override
    public String getSchema() {
        return "double voltage;double totalEnergyJoules;double totalPowerWatts;double totalCurrent";
    }

    @Override
    public PowerDistribution unpack(ByteBuffer bb) {
        throw new UnsupportedOperation("PowerDistribution can't be unpacked from a byte buffer!");
    }

    @Override
    public void pack(ByteBuffer bb, PowerDistribution value) {
        bb.putDouble(value.getVoltage());
        bb.putDouble(value.getTotalEnergy());
        bb.putDouble(value.getTotalPower());
        bb.putDouble(value.getTotalCurrent());
    }
    
    /**
     * Static instance of this.
     */
    public static PowerDistributionStruct struct = new PowerDistributionStruct();
}
