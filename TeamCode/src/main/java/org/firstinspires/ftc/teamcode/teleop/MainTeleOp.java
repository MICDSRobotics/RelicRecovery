/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robotplus.gamepadwrapper.Controller;
import org.firstinspires.ftc.teamcode.robotplus.hardware.GrabberPrimer;
import org.firstinspires.ftc.teamcode.robotplus.hardware.MecanumDrive;
import org.firstinspires.ftc.teamcode.robotplus.hardware.Robot;
import org.firstinspires.ftc.teamcode.robotplus.robodata.AccessControl;

import static org.firstinspires.ftc.teamcode.robotplus.gamepadwrapper.Controller.Button.PRESSED;


/**
 * Main Teleop Program for Competition
 * @author Blake A, Alex M
 * @since 1/4/2018
 */

@TeleOp(name="Drive Robot", group="Competition OpModes")
//@Disabled
public class MainTeleOp extends OpMode
{

    private ElapsedTime runtime = new ElapsedTime();

    private Robot robot;

    private Controller p1;
    private Controller p2;

    private MecanumDrive drivetrain;

    private DcMotor raiser;
    private Servo grabber;
    private Servo armRotator;
    private Servo armExtender;
    private GrabberPrimer grabberPrimer;

    private AccessControl accessControl = new AccessControl();

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        p1 = new Controller(gamepad1);
        p2 = new Controller(gamepad2);

        robot = new Robot(hardwareMap);
        drivetrain = (MecanumDrive) robot.getDrivetrain();

        raiser = hardwareMap.dcMotor.get("raiser");
        grabber = hardwareMap.servo.get("grabber");
        grabberPrimer = new GrabberPrimer(grabber);

        grabber.scaleRange(0.25, 1.0);

        armRotator = hardwareMap.servo.get("armRotator");
        armExtender = hardwareMap.servo.get("armExtender");

        armRotator.scaleRange(0.1,0.9);
        armExtender.scaleRange(0.16, 0.85);

        raiser.setDirection(DcMotorSimple.Direction.FORWARD);

    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {

        telemetry.addData("Status", "Running: " + runtime.toString());
        telemetry.addData("Access", accessControl.getTelemetryState());

        if (accessControl.isG2Primary()) {
            drivetrain.complexDrive(gamepad2, telemetry);
        }
        else {
            drivetrain.complexDrive(gamepad1, telemetry);
        }

        if (p1.start == PRESSED || p2.start == PRESSED) {
            accessControl.changeAccess();
        }

        //Raise arm while the y button is held, lower it when a it held
        if(p1.a.isDown() || p2.a.isDown()){
            raiser.setPower(1);
        } else if (p1.b.isDown() || p2.b.isDown()) {
            raiser.setPower(-1);
        } else {
            raiser.setPower(0);
        }

        //Set grabber position
        if(p1.leftBumper == PRESSED || p2.leftBumper == PRESSED){
            grabberPrimer.open();
        } else if (p1.rightBumper == PRESSED || p2.rightBumper == PRESSED){
            grabberPrimer.grab();
        }

        //Set rotation servo positions
        if(p1.dpadLeft.isDown() || p2.dpadLeft.isDown()){
            armRotator.setPosition(Math.min(1, armRotator.getPosition() + 0.01));
        } else if (p1.dpadRight.isDown() || p2.dpadRight.isDown()){
            armRotator.setPosition(Math.max(0, armRotator.getPosition() - 0.01));
        }

        //Set extender servo positions
        if(p1.dpadUp.isDown() || p2.dpadUp.isDown()){
            armExtender.setPosition(Math.min(1, armExtender.getPosition() + 0.01));
        } else if(p1.dpadDown.equals(Controller.Button.HELD) || p2.dpadDown.equals(Controller.Button.HELD)){
            armExtender.setPosition(Math.max(0, armExtender.getPosition() - 0.01));
        }

        telemetry.addData("Grabber Position", grabber.getPosition());

        telemetry.addData("ArmRotator Position", armRotator.getPosition());
        telemetry.addData("ArmExtender Position", armExtender.getPosition());

        p1.update();
        p2.update();

    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        robot.stopMoving();
    }

}
