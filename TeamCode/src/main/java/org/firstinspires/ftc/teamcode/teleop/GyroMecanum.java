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
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.robotplus.gamepadwrapper.Controller;
import org.firstinspires.ftc.teamcode.robotplus.hardware.ComplexRaiser;
import org.firstinspires.ftc.teamcode.robotplus.hardware.FlipperIntake;
import org.firstinspires.ftc.teamcode.robotplus.hardware.IMUWrapper;
import org.firstinspires.ftc.teamcode.robotplus.hardware.MecanumDrive;
import org.firstinspires.ftc.teamcode.robotplus.hardware.Robot;
import org.firstinspires.ftc.teamcode.robotplus.robodata.AccessControl;

import static org.firstinspires.ftc.teamcode.robotplus.gamepadwrapper.Controller.Button.PRESSED;


/**
 * Main Teleop Program for Competition
 * @author Blake A, Alex M
 * @since 1/4/2018
 */

@TeleOp(name="Mecanum Robot", group="Competition OpModes")
//@Disabled
public class GyroMecanum extends OpMode
{

    private int counts;
    private boolean toggleboi;

    private ElapsedTime runtime = new ElapsedTime();

    private boolean lowSpeed = false;

    private Robot robot;

    private Controller p1;
    private Controller p2;

    private MecanumDrive drivetrain;

    private ComplexRaiser raiser;
    private FlipperIntake intake;
    private IMUWrapper imuWrapper;

    private Servo armRotator;
    private Servo armExtender;

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
        imuWrapper = new IMUWrapper(hardwareMap);

        armRotator = hardwareMap.servo.get("armRotator");
        armExtender = hardwareMap.servo.get("armExtender");

        armRotator.scaleRange(0.158, 0.7);
        armExtender.scaleRange(0.16, 0.95);

        counts = 0;
        toggleboi = false;

        intake.getIntake().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //armExtender.setPosition(0.8);
        //armRotator.setPosition(0.5);
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

        //Drivetrain switching & low speed
        if (accessControl.isG2Primary()) {
            if (this.lowSpeed) {
                //drivetrain.complexDrive(gamepad2, telemetry, 0.5);
                drivetrain.gyroDrive(gamepad2, telemetry, imuWrapper.getOrientation().toAngleUnit(AngleUnit.RADIANS).firstAngle, 0.5);
            }
            else {
                //drivetrain.complexDrive(gamepad2, telemetry);
                drivetrain.gyroDrive(gamepad2, telemetry, imuWrapper.getOrientation().toAngleUnit(AngleUnit.RADIANS).firstAngle);
            }
        }
        else {
            if (this.lowSpeed) {
                //drivetrain.complexDrive(gamepad1, telemetry, 0.5);
                drivetrain.gyroDrive(gamepad1, telemetry, imuWrapper.getOrientation().toAngleUnit(AngleUnit.RADIANS).firstAngle, 0.5);
            }
            else {
                //drivetrain.complexDrive(gamepad1, telemetry);
                drivetrain.gyroDrive(gamepad1, telemetry, imuWrapper.getOrientation().toAngleUnit(AngleUnit.RADIANS).firstAngle);
            }
        }

        if (p1.start == PRESSED || p2.start == PRESSED) {
            accessControl.changeAccess();
        }

        telemetry.addData("Access", accessControl.getTelemetryState());

        if (p1.x == PRESSED || p2.x == PRESSED) {
            this.lowSpeed = !this.lowSpeed;
        }

        telemetry.addData("Slowmode", this.lowSpeed);

        //Raise outtake while the y button is held, lower it when a it held
        if(p1.a.isDown() || p2.a.isDown()){
            raiser.raiseUp();
        } else if (p1.b.isDown() || p2.b.isDown()) {
            raiser.lower();
        } else {
            raiser.stop();
        }

        //Set arm rotation servo positions
        if(p1.dpadLeft.isDown() || p2.dpadLeft.isDown()){
            armRotator.setPosition(Math.min(1, armRotator.getPosition() + 0.05));
        } else if (p1.dpadRight.isDown() || p2.dpadRight.isDown()){
            armRotator.setPosition(Math.max(0, armRotator.getPosition() - 0.05));
        }

        telemetry.addData("ArmRotator Position", armRotator.getPosition());

        telemetry.addData("ArmExtender Position", armExtender.getPosition());

        // p1 shifted controls
        if(p1.y.isDown()){

            // outtake stuff
            if (p1.leftBumper.isDown()) {
                raiser.retractFlipper();
            }
            if (p1.rightBumper.isDown()) {
                raiser.outtakeGlyph();
            }

            // clear intake if in bad situation
            if (p1.x.isDown()) {
                this.intake.reverseIntake();
            }

            // recalibrate IMU
            if (p1.b == PRESSED) {
                imuWrapper.getIMU().initialize(imuWrapper.getInitilizationParameters());
                telemetry.addData("reset", "Trying to recalibrate");
            }

            if(p1.dpadUp.isDown()){
                intake.flipOutIntake();
            }

            if(p1.dpadDown.isDown()){
                intake.flipInIntake();
            }


        } else {

            // intake stuff
            if (p1.leftBumper == PRESSED) {
                if (toggleboi) { // TODO: fix the current position
                    intake.flipOutIntake();
                } else {
                    intake.flipInIntake();
                }
                toggleboi = !toggleboi;
            }
            if (p1.rightBumper == PRESSED) {
                if (intake.getIntake().getPower() >= 0) {
                    intake.startIntake();
                } else {
                    intake.stopIntake();
                }
            }

            //Set arm extender servo positions
            if(p1.dpadUp.isDown()){
                armExtender.setPosition(Math.min(1, armExtender.getPosition() + 0.05));
            } else if(p1.dpadDown.isDown()){
                armExtender.setPosition(Math.max(0, armExtender.getPosition() - 0.05));
            }

        }

        //p2 shifted controls
        if(p2.y.isDown()){

            // outtake stuff
            if (p2.leftBumper.isDown()) {
                raiser.retractFlipper();
            }
            if (p2.rightBumper.isDown()) {
                raiser.outtakeGlyph();
            }

            // clear intake if in bad situation
            if (p2.x.isDown()) {
                this.intake.reverseIntake();
            }

            if(p2.dpadUp.isDown()){
                intake.flipOutIntake();
            }

            if(p2.dpadDown.isDown()){
                intake.flipInIntake();
            }

            // recalibrate IMU
            if (p2.b == PRESSED) {
                imuWrapper.getIMU().initialize(imuWrapper.getInitilizationParameters());
                telemetry.addData("reset", "Trying to recalibrate");
            }

        } else {

            // intake stuff
            if (p2.leftBumper == PRESSED) {
                if (intake.getRotation().getPosition() < 100) { // TODO: fix the current position bound
                    intake.flipOutIntake();
                } else {
                    intake.flipInIntake();
                }
            }
            if (p2.rightBumper == PRESSED) {
                if (intake.getIntake().getPower() >= 0) {
                    intake.startIntake();
                } else {
                    intake.stopIntake();
                }
            }

            //Set arm extender servo positions
            if(p2.dpadUp.isDown()){
                armExtender.setPosition(Math.min(1, armExtender.getPosition() + 0.05));
            } else if(p2.dpadDown.isDown()){
                armExtender.setPosition(Math.max(0, armExtender.getPosition() - 0.05));
            }

        }

        telemetry.addData("Intake Motors", this.intake.getIntake().getPower());
        telemetry.addData("Intake Flipper", intake.getRotation().getPosition());

        telemetry.update();
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
