package org.firstinspires.ftc.teamcode.compautonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.teamcode.robotplus.autonomous.TimeOffsetVoltage;
import org.firstinspires.ftc.teamcode.robotplus.autonomous.VuforiaWrapper;
import org.firstinspires.ftc.teamcode.robotplus.hardware.ColorSensorWrapper;
import org.firstinspires.ftc.teamcode.robotplus.hardware.ComplexRaiser;
import org.firstinspires.ftc.teamcode.robotplus.hardware.FlipperIntake;
import org.firstinspires.ftc.teamcode.robotplus.hardware.GrabberPrimer;
import org.firstinspires.ftc.teamcode.robotplus.hardware.IMUWrapper;
import org.firstinspires.ftc.teamcode.robotplus.hardware.MecanumDrive;
import org.firstinspires.ftc.teamcode.robotplus.hardware.Robot;

/**
 * Red Right Scenario
 * @author Alex Migala, Blake Abel
 * @since 12/30/17
 */
@Autonomous(name="RedRight", group="compauto")
public class RedRight extends LinearOpMode implements Settings {

    private Robot robot;
    private MecanumDrive drivetrain;

    private ComplexRaiser raiser;
    private FlipperIntake intake;

    private IMUWrapper imuWrapper;
    private VuforiaWrapper vuforiaWrapper;

    private double voltage;

    private Servo armExtender;
    private Servo armRotator;
    private ColorSensorWrapper colorSensorWrapper;

    private RelicRecoveryVuMark relicRecoveryVuMark;

    @Override
    public void runOpMode() throws InterruptedException {

        //Initialize hardware
        robot = new Robot(hardwareMap);
        drivetrain = (MecanumDrive) robot.getDrivetrain();

        raiser = new ComplexRaiser(hardwareMap);
        intake = new FlipperIntake(hardwareMap);

        imuWrapper = new IMUWrapper(hardwareMap);
        vuforiaWrapper = new VuforiaWrapper(hardwareMap);

        armRotator = hardwareMap.servo.get("armRotator");
        armExtender = hardwareMap.servo.get("armExtender");

        //Prepare hardware
        armRotator.scaleRange(0.158, 0.7);
        armExtender.scaleRange(0.16, 0.95);

        // set the hardware to position
        armExtender.setPosition(1.0);
        armRotator.setPosition(1.0);
        raiser.retractFlipper();

        colorSensorWrapper = new ColorSensorWrapper(hardwareMap);

        vuforiaWrapper.getLoader().getTrackables().activate();

        telemetry.update();

        waitForStart();

        intake.flipOutIntake();

        //STEP 1: Scan vuforia pattern
        relicRecoveryVuMark = Common.scanVuMark(this, vuforiaWrapper);

        this.armExtender.setPosition(0.75);
        sleep(500);
        this.armRotator.setPosition(0.59);

        //STEP 2: Hitting the jewel
        Common.hitJewel(this, armRotator, armExtender, colorSensorWrapper, false);

        sleep(1000);

        // move backwards and slam into the wall
        this.drivetrain.complexDrive(MecanumDrive.Direction.DOWN.angle(), 1, 0);
        // 115cm
        double voltage = hardwareMap.voltageSensor.get("Expansion Hub 1").getVoltage();
        sleep(TimeOffsetVoltage.calculateDistance(voltage, 40)); // 115cm
        this.drivetrain.stopMoving();
        this.intake.stopIntake();
        sleep(100);

        // TODO: Investigate using not full velocity and see if that will allow the robot to move forward but not onto the stone
        this.drivetrain.complexDrive(MecanumDrive.Direction.UP.angle(), 0.5, 0);
        sleep(150);
        this.drivetrain.stopMoving();
        sleep(1000);

        //THIS IS THE DISTANCE TO BE CHANGED
        this.drivetrain.complexDrive(MecanumDrive.Direction.RIGHT.angle(), 1, 0);
        sleep(sideShort - 200); // -100
        this.drivetrain.stopMoving();
        sleep(500);

        drivetrain.setAngle(imuWrapper, 0);

        //Lower raiser a bit
        this.raiser.lower();
        sleep(500);
        this.raiser.stop();

        Common.faceCorrectColumn(this, drivetrain, relicRecoveryVuMark, imuWrapper);

        telemetry.update();

        drivetrain.stopMoving();
        sleep(500);

        drivetrain.complexDrive(MecanumDrive.Direction.DOWN.angle(), slamIntoWallSpeed, 0);
        sleep(distanceToWall);
        this.drivetrain.stopMoving();
        sleep(750);
        raiser.outtakeGlyph();
        sleep(700);

        drivetrain.stopMoving();
        sleep(200);

        Common.wiggle(this, drivetrain);

        this.drivetrain.complexDrive(MecanumDrive.Direction.DOWN.angle(), 1, 0);
        sleep(distanceToWall + 150);
        this.drivetrain.stopMoving();

        // pull out
        this.drivetrain.complexDrive(MecanumDrive.Direction.UP.angle(), 1, 0);
        sleep(300);
        this.drivetrain.stopMoving();

        telemetry.update();

        sleep(500);

        //Move back to center of cryptobox tape (if necessary)
        // START
        /*moveToCorrectColumn();

        raiser.lower();
        sleep(450);
        raiser.stop();

        this.attemptToGetMultiBlock();

        this.drivetrain.stopMoving();
        sleep(200);

        //REPEAT PUTTING BLOCK IN THE THING
        //Turn 90 degrees to face cryptobox
        drivetrain.setAngle(imuWrapper, Math.PI/2);
        sleep(500);

        moveToCorrectColumn();

        telemetry.update();

        drivetrain.complexDrive(MecanumDrive.Direction.UP.angle(), slamIntoWallSpeed, 0);
        sleep(200);

        raiser.outtakeGlyph();
        drivetrain.stopMoving();
        sleep(500);
        raiser.retractFlipper();

        // PULL OUT (Once)
        this.drivetrain.complexDrive(MecanumDrive.Direction.DOWN.angle(), 1, 0);
        sleep(150);
        this.drivetrain.stopMoving();
        */
    }
}