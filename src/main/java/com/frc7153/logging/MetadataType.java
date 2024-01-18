package com.frc7153.logging;

/** 
 * Different types of metadata, used for how data is sorted in AdvantageScope's
 * Metadata tab.
 */
public enum MetadataType {
    /** Timestamp of robot startup (1st) */
    TIMESTAMP_STARTUP('A'), 

    /** Timestamp of network-based startup (ie, FMS or DS) (2nd) */
    TIMESTAMP_NETWORK('B'), 

    /** Generic timestamp (3rd) */
    TIMESTAMP('C'),
    
    /** Info about current match (4th) */
    MATCH_INFO('D'), 

    /** Info about RoboRIO (5th) */
    ROBORIO('E'),

    /** Other (last) */
    OTHER('Z');

    // Prefix attached to Metadata entries
    public final char prefix;
    private MetadataType(char prefix) { this.prefix = prefix; }
}