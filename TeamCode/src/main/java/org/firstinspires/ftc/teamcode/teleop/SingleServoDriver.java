package org.firstinspires.ftc.teamcode.teleop;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.robotplus.gamepadwrapper.Controller;

/**
 * Created by amigala on 3/1/2018.
 */
@Disabled
@TeleOp(name = "SingleServo", group = "Test")
public class SingleServoDriver extends OpMode {
    private Servo singleServo;
    private Controller p1;

    @Override
    public void init() {
        this.singleServo = hardwareMap.get(Servo.class, "servo");
        this.p1 = new Controller(gamepad1);
    }

    @Override
    public void loop() {
        if (p1.dpadUp.isDown()) {
            this.singleServo.setPosition(this.singleServo.getPosition() + 0.01);
        }
        else if (p1.dpadDown.isDown()) {
            this.singleServo.setPosition(this.singleServo.getPosition() - 0.01);
        }

        telemetry.addData("Position", singleServo.getPosition());

        p1.update();
    }
}
