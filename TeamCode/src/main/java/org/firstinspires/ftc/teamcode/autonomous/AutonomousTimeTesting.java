package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.robotplus.autonomous.TimeOffsetVoltage;
import org.firstinspires.ftc.teamcode.robotplus.hardware.*;

import org.firstinspires.ftc.teamcode.robotplus.hardware.MecanumDrive;

import java.sql.Time;

/**
 * Created by amigala on 1/22/2018.
 */

@Autonomous(name = "TimeTesting", group = "Testing")
public class AutonomousTimeTesting extends LinearOpMode {
    private MecanumDrive drive;
    private Robot robot;

    @Override
    public void runOpMode() {
        this.robot = new Robot(hardwareMap);
        this.drive = (MecanumDrive) this.robot.getDrivetrain();

        waitForStart();

        // begin moving
        //this.drive.complexDrive(MecanumDrive.Direction.UP.angle(), 1, 0);

        double voltage = hardwareMap.voltageSensor.get("Expansion Hub 1").getVoltage();
        telemetry.addData("Voltage", hardwareMap.voltageSensor.get("Expansion Hub 1").getVoltage());
        telemetry.addData("Output", TimeOffsetVoltage.calculateDistance(voltage, 100, 1));

        telemetry.update();

        this.drive.complexDrive(MecanumDrive.Direction.UP.angle(), 1, 0);

        //sleep((long)TimeOffsetVoltage.calculateDistance(hardwareMap.voltageSensor.get("Expansion Hub 1").getVoltage(), 100));

        this.drive.stopMoving();
        telemetry.update();
        sleep(5000);
    }
}
