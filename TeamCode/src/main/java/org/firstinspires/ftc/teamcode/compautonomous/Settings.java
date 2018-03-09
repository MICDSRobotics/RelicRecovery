package org.firstinspires.ftc.teamcode.compautonomous;

/**
 * Settings for the autocomp. Times are in ms
 * @author Alex M, Blake A
 * @since 12/30/17
 */
public interface Settings {

    //Full battery tests
    int firstStretch = 1300;
    int forwardShort = 950;
    int sideShort = 1000; // 750
    int rotate90 = 475;
    //int lineUp = ;
    double slamIntoWallSpeed = 0.4;
    int distanceToWall = 350;


    /* FIRST TEST VALUES (FULL BATTERY?)
    int firstStretch = 1400;
    int forwardShort = 1050;
    int sideShort = 900;
    int secondStretch = 1000;
    int rotate90 = 549; //2196 for a full rotation
    double slamIntoWallSpeed = 0.4;
    */

    /**
     * These values are used for DogeCV
     */

    int distanceToCenter = 1; // this is the threshold for how much the robot should move to get to the center column
}
