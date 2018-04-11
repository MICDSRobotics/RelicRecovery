package org.firstinspires.ftc.teamcode.compautonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
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
 * Blue Left Scenario
 * @author Alex M, Blake A
 * @since 12/30/2017
 */

@Autonomous(name="BlueLeft", group="compauto")
public class BlueLeft extends LinearOpMode implements Settings{

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
        armRotator.setPosition(0.75);
        intake.flipInIntake();
        raiser.retractFlipper();

        colorSensorWrapper = new ColorSensorWrapper(hardwareMap);

        vuforiaWrapper.getLoader().getTrackables().activate();

        raiser.retractFlipper();

        telemetry.update();

        waitForStart();

        //STEP 1: Scan vuforia pattern
        relicRecoveryVuMark = Common.scanVuMark(this, vuforiaWrapper);

        intake.flipOutIntake();

        //STEP 2: Hitting the jewel
        Common.hitJewel(this, armRotator, armExtender, colorSensorWrapper, true);

        sleep(1000);

        this.intake.flipInIntake();
        // STEP 3: MOVE OFF BALANCING STONE
        this.drivetrain.complexDrive(MecanumDrive.Direction.UP.angle(), 1, 0);
        // 115cm
        double voltage = hardwareMap.voltageSensor.get("Expansion Hub 1").getVoltage();
        sleep((long) TimeOffsetVoltage.calculateDistance(voltage, 170));
        this.drivetrain.stopMoving();
        this.intake.stopIntake();
        sleep(100);

        robot.stopMoving();
        sleep(1000);

        drivetrain.setAngle(imuWrapper, 0);
        sleep(1000);

        drivetrain.complexDrive(MecanumDrive.Direction.DOWN.angle(), 1, 0);
        sleep(750);

        robot.stopMoving();
        sleep(1000);

        //STEP 4: MOVE TOWARDS CRYPTOBOX
        drivetrain.complexDrive(MecanumDrive.Direction.RIGHT.angle(), 1,0);
        sleep(sideShort - 100);

        robot.stopMoving();
        sleep(1000);

        drivetrain.setAngle(imuWrapper, Math.PI);
        sleep(1000);

        //Lower raiser a bit
        this.raiser.lower();
        sleep(500);
        this.raiser.stop();

        //STEP 5: SCORE GLYPH INTO CORRECT COLUMN
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

    }

}
