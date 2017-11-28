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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.robotplus.gamepadwrapper.ControllerWrapper;
import org.firstinspires.ftc.teamcode.robotplus.hardware.MecanumDrive;
import org.firstinspires.ftc.teamcode.robotplus.hardware.Robot;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Test Robot Hardware", group="Test")
//@Disabled
public class TestRobotHardware extends OpMode
{

    private ElapsedTime runtime = new ElapsedTime();

    private Robot robot;

    private ControllerWrapper game1;

    private MecanumDrive drivetrain;

    private DcMotor raiser;
    private DcMotor grabber;
    private Servo armRotator;
    private Servo armExtender;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        robot = new Robot(hardwareMap);
        game1 = new ControllerWrapper(gamepad1);
        drivetrain = (MecanumDrive) robot.getDrivetrain();

        raiser = hardwareMap.dcMotor.get("raiser");
        grabber = hardwareMap.dcMotor.get("grabber");

        armRotator = hardwareMap.servo.get("armRotator");
        armExtender = hardwareMap.servo.get("armExtender");

        raiser.setDirection(DcMotorSimple.Direction.REVERSE);
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

        drivetrain.complexDrive(gamepad1, telemetry);

        //Raise arm while the y button is held, lower it when a it held
        if(gamepad1.y){
            raiser.setPower(1);
        } else if (gamepad1.a) {
            raiser.setPower(-1);
        } else {
            raiser.setPower(0);
        }

        //Increment servos by small amounts when bumper is held.
        /*if(gamepad1.left_bumper){
            grabber.setPower(-0.5);
        } else if (gamepad1.right_bumper){
            grabber.setPower(0.5);
        } else {
            grabber.setPower(0);
        }*/

        if (gamepad1.right_bumper) {
            grabber.setPower(0.5);
        }

        // lock grabber
        if(gamepad1.left_trigger > 0) {
            grabber.setPower(-0.5);
        }

        if (gamepad1.right_bumper) {
            robot.stopMoving();
            drivetrain.stopMoving();
            grabber.setPower(0);
        }

        //Set rotation servo positions
        if(gamepad1.dpad_left){
            armRotator.setPosition(Math.min(1, armRotator.getPosition() + 0.01));
        } else if (gamepad1.dpad_right){
            armRotator.setPosition(Math.max(0, armRotator.getPosition() - 0.01));
        }

        //Set extender servo positions
        if(gamepad1.dpad_up){
            armExtender.setPosition(Math.min(1, armExtender.getPosition() + 0.01));
        } else if(gamepad1.dpad_down){
            armExtender.setPosition(Math.max(0, armExtender.getPosition() - 0.01));
        }

        telemetry.addData("Grabber Power", grabber.getPower());

        telemetry.addData("ArmRotator Position", armRotator.getPosition());
        telemetry.addData("ArmExtender Position", armExtender.getPosition());


        /*
        if (gamepad1.a) {
            // cache the gamepad a button
            game1.getLetterInterface().getAButton().setPress(true);
        }
        else if (game1.getLetterInterface().getAButton().isPressed()) {
            // do something
            // set the interface back
            game1.getLetterInterface().getAButton().setPress(false);
        }
        */

        telemetry.addData("Main1", drivetrain.getmajorDiagonal().getMotor1().getPower());
        telemetry.addData("Minor1", drivetrain.getMinorDiagonal().getMotor1().getPower());
        telemetry.addData("Minor2", drivetrain.getMinorDiagonal().getMotor2().getPower());
        telemetry.addData("Main2", drivetrain.getmajorDiagonal().getMotor2().getPower());


    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}
