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
import org.firstinspires.ftc.teamcode.robotplus.hardware.Outtake;
import org.firstinspires.ftc.teamcode.robotplus.hardware.Robot;

/**
 * Red Left Scenario
 * @author Alex M, Blake A
 * @since 12/30/17
 */

@Autonomous(name="RedLeft", group="compauto")
public class RedLeft extends LinearOpMode implements Settings {

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
        //intake.flipInIntake();
        //raiser.retractFlipper();

        colorSensorWrapper = new ColorSensorWrapper(hardwareMap);

        vuforiaWrapper.getLoader().getTrackables().activate();

        raiser.retractFlipper();

        telemetry.update();

        waitForStart();

        intake.flipOutIntake();

        //STEP 1: Scan vuforia pattern
        relicRecoveryVuMark = Common.scanVuMark(this, vuforiaWrapper);

        //STEP 2: Hitting the jewel
        Common.hitJewel(this, armRotator, armExtender, colorSensorWrapper, false);

        sleep(1000);

        // Move backwards off balancing stone
        //this.intake.flipInIntake();
        this.drivetrain.complexDrive(MecanumDrive.Direction.DOWN.angle(), 1, 0); // move backwards
        double voltage = hardwareMap.voltageSensor.get("Expansion Hub 1").getVoltage();
        sleep(TimeOffsetVoltage.calculateDistance(voltage, 55));
        this.drivetrain.stopMoving();
        this.intake.stopIntake();
        sleep(100);

        //Turn 90 degrees to face cryptobox
        drivetrain.setAngle(imuWrapper, -Math.PI/2);
        sleep(500);

        /*Lower raiser a bit
        this.raiser.lower();
        sleep(500);
        this.raiser.stop();
        */

        Common.faceCorrectColumn(this, drivetrain, relicRecoveryVuMark, imuWrapper);

        telemetry.update();

        /*this.raiser.raiseUp();
        sleep(750);
        this.raiser.stop();*/

        drivetrain.stopMoving();
        //sleep(distanceToWall);

        //drivetrain.complexDrive(MecanumDrive.Direction.DOWN.angle(), slamIntoWallSpeed, 0);
        //sleep(200);
        raiser.outtakeGlyph();
        sleep(1000);

        //drivetrain.stopMoving();
        //sleep(500);

        //Common.wiggle(this, drivetrain);

        this.drivetrain.complexDrive(MecanumDrive.Direction.DOWN.angle(), 1, 0);
        sleep(distanceToWall + 150);
        this.drivetrain.stopMoving();

        Common.wiggle(this, this.drivetrain);
        Common.wiggle(this, this.drivetrain);

        // pull out
        this.drivetrain.complexDrive(MecanumDrive.Direction.UP.angle(), 1, 0);
        sleep(500);
        this.drivetrain.stopMoving();

        telemetry.update();

        /*this.raiser.lower();
        sleep(750);
        this.raiser.stop();*/

        sleep(500);

        // TODO: REMOVE IF NOT WORK THIS IS PSEUDO CODE
        Common.attemptToGetMultiBlock(this, drivetrain, imuWrapper, intake, raiser, hardwareMap);
    }
}
