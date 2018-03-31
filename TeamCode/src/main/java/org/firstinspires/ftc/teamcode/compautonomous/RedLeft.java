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

        armExtender.setPosition(1.0);
        armRotator.setPosition(1.0);

        colorSensorWrapper = new ColorSensorWrapper(hardwareMap);

        vuforiaWrapper.getLoader().getTrackables().activate();

        raiser.retractFlipper();

        telemetry.update();

        waitForStart();

        //STEP 1: Scan vuforia pattern
        relicRecoveryVuMark = Common.scanVuMark(this, vuforiaWrapper);

        //STEP 2: Hitting the jewel
        Common.hitJewel(this, armRotator, armExtender, colorSensorWrapper, false);

        sleep(1000);

        // Move backwards off balancing stone
        this.intake.flipInIntake();
        this.drivetrain.complexDrive(MecanumDrive.Direction.DOWN.angle(), 1, 0); // move backwards
        // 160cm
        double voltage = hardwareMap.voltageSensor.get("Expansion Hub 1").getVoltage();
        sleep((long)TimeOffsetVoltage.calculateDistance(voltage, 175));
        this.drivetrain.stopMoving();
        this.intake.stopIntake();
        sleep(100);

        //Turn 90 degrees to face cryptobox
        drivetrain.setAngle(imuWrapper, -Math.PI/2);
        sleep(500);

        //Lower raiser a bit
        this.raiser.lower();
        sleep(500);
        this.raiser.stop();

        Common.faceCorrectColumn(this, drivetrain, relicRecoveryVuMark, imuWrapper);

        telemetry.update();

        drivetrain.stopMoving();
        sleep(distanceToWall);

        drivetrain.complexDrive(MecanumDrive.Direction.DOWN.angle(), slamIntoWallSpeed, 0);
        sleep(500);
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

        // TODO: REMOVE IF NOT WORK THIS IS PSEUDO CODE
        /*this.attemptToGetMultiBlock();
        sleep(500);
        //this.moveToCorrectColumn();
        this.drivetrain.complexDrive(MecanumDrive.Direction.DOWN.angle(), 1, 0);
        sleep(350);
        this.drivetrain.stopMoving();
        // pull out
        this.drivetrain.complexDrive(MecanumDrive.Direction.UP.angle(), 1, 0);
        sleep(300);
        this.drivetrain.stopMoving();*/
    }

    public void attemptToGetMultiBlock() {
        // Face glyph horde using the gyro
        this.drivetrain.setAngle(this.imuWrapper, -Math.PI/2);
        // briefly ram into the block pile
        this.drivetrain.complexDrive(MecanumDrive.Direction.UP.angle(), 1, 0);
        sleep(1000);
        this.drivetrain.stopMoving();
        sleep(500);

        // Attempt to pick up a block
        intake.startIntake();
        sleep(1000);
        drivetrain.complexDrive(MecanumDrive.Direction.DOWN.angle(), 0.5, 0);
        sleep(100);
        drivetrain.stopMoving();

        //Raise up
        raiser.raiseUp();
        sleep(200);
        raiser.stop();
        sleep(200);
        // Move back towards cryptobox
        this.drivetrain.complexDrive(MecanumDrive.Direction.DOWN.angle(), 1, 0);
        sleep(TimeOffsetVoltage.calculateDistance(hardwareMap.voltageSensor.get("Expansion Hub 1").getVoltage(), 110));
        drivetrain.stopMoving();
    }
}
