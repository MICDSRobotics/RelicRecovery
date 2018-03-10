package org.firstinspires.ftc.teamcode.autonomous.lib;

/**
 * ProfileMap
 * @since 2/28/18
 * @author Alex M, Blake A
 */
public enum ProfileMap {
    RELIC, MAIN;

    /**
     * Stringifies the name of the current map
     * @return the name in String format
     * @see String
     */
    public String stringify() {
        return String.valueOf(this);
    }
}