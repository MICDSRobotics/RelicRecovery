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
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.autonomous.lib.ProfileMap;
import org.firstinspires.ftc.teamcode.robotplus.gamepadwrapper.Controller;
import org.firstinspires.ftc.teamcode.robotplus.hardware.ComplexRaiser;
import org.firstinspires.ftc.teamcode.robotplus.hardware.FlipperIntake;
import org.firstinspires.ftc.teamcode.robotplus.hardware.GrabberPrimer;
import org.firstinspires.ftc.teamcode.robotplus.hardware.MecanumDrive;
import org.firstinspires.ftc.teamcode.robotplus.hardware.Outtake;
import org.firstinspires.ftc.teamcode.robotplus.hardware.Robot;
import org.firstinspires.ftc.teamcode.robotplus.robodata.AccessControl;

import static org.firstinspires.ftc.teamcode.robotplus.gamepadwrapper.Controller.Button.PRESSED;


/**
 * Main Teleop Program for Competition (with Relic Grabber
 * @author Blake A, Alex M
 * @since 1/4/2018
 */

@TeleOp(name="Relic Grabbing", group="Gogo gadget extendo arm")
//@Disabled
public class RelicTeleOp extends OpMode
{

    private ElapsedTime runtime = new ElapsedTime();

    private boolean lowSpeed = false;

    private Robot robot;

    private Controller p1;
    private Controller p2;

    private MecanumDrive drivetrain;

    private ComplexRaiser raiser;
    private FlipperIntake intake;

    private Servo armRotator;
    private Servo armExtender;

    private CRServo grabberExtender;
    private Servo grabberWrist;
    private Servo grabberHand;

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

        raiser = new ComplexRaiser(hardwareMap);
        intake = new FlipperIntake(hardwareMap);

        armRotator = hardwareMap.servo.get("armRotator");
        armExtender = hardwareMap.servo.get("armExtender");

        armRotator.scaleRange(0.1,0.9);
        armExtender.scaleRange(0.16, 0.85);

        grabberExtender = hardwareMap.crservo.get("grabberExtender");
        grabberWrist = hardwareMap.servo.get("grabberWrist");
        grabberHand = hardwareMap.servo.get("grabberHand");
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

        //Drivetrain switching & low speed
        if (accessControl.isG2Primary()) {
            if (this.lowSpeed) {
                drivetrain.complexDrive(gamepad2, telemetry, 0.5);
            }
            else {
                drivetrain.complexDrive(gamepad2, telemetry);
            }
        }
        else {
            if (this.lowSpeed) {
                drivetrain.complexDrive(gamepad1, telemetry, 0.5);
            }
            else {
                drivetrain.complexDrive(gamepad1, telemetry);
            }
        }

        if (p1.start == PRESSED || p2.start == PRESSED) {
            accessControl.changeAccess();
        }

        if (p1.x == PRESSED || p2.x == PRESSED) {
            this.lowSpeed = !this.lowSpeed;
            telemetry.addData("Switching", "YEET");
        }

        //Raise arm while the y button is held, lower it when a it held
        if(p1.a.isDown() || p2.a.isDown()){
            raiser.raiseUp();
        } else if (p1.b.isDown() || p2.b.isDown()) {
            raiser.lower();
        } else {
            raiser.stop();
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

        grabberExtender.setPower(gamepad2.right_trigger);
        grabberExtender.setPower(-gamepad2.left_trigger);

        // p1 shifted controls
        if(p1.y.isDown()){

            // outtake stuff

            if (p1.leftBumper.isDown()) {
                raiser.retractFlipper();
            }

            if (p1.rightBumper.isDown()) {
                raiser.purgeGlyph();
            }

        } else {

            // intake stuff

            if (p1.leftBumper.isDown()) {
                if (intake.getRotation().getPosition() < 0) {
                    intake.flipOutIntake();
                } else {
                    intake.flipInIntake();
                }
            }

            if (p1.rightBumper.isDown()) {
                if (intake.getIntake().getFrontPower() < 0) {
                    intake.startIntake();
                } else {
                    intake.stopIntake();
                }
            }

        }

        //p2 shifted controls
        if(p2.y.isDown()){

            if(p2.leftBumper.isDown()) {
                grabberHand.setPosition(Math.min(1, grabberHand.getPosition() + 0.01));
            }

            if(p2.rightBumper.isDown()){
                grabberWrist.setPosition(Math.min(1, grabberWrist.getPosition() + 0.01));
            }

        } else {

            if(p2.leftBumper.isDown()) {
                grabberHand.setPosition(Math.max(0, grabberHand.getPosition() - 0.01));
            }

            if(p2.rightBumper.isDown()){
                grabberWrist.setPosition(Math.max(0, grabberWrist.getPosition() - 0.01));
            }

        }
        
        telemetry.addData("Slowmode", this.lowSpeed);

        telemetry.addData("ArmRotator Position", armRotator.getPosition());
        telemetry.addData("ArmExtender Position", armExtender.getPosition());

        telemetry.addData("Grabber Extender Power", grabberExtender.getPower());

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
