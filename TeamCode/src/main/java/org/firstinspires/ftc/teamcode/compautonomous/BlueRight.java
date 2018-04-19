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
 * Blue Right Scenario
 * @author Alex M, Blake A
 * @since 12/30/17
 */

@Autonomous(name="BlueRight", group="compauto")
public class BlueRight extends LinearOpMode implements Settings{

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
        //intake.flipInIntake();
        //raiser.retractFlipper();

        colorSensorWrapper = new ColorSensorWrapper(hardwareMap);

        vuforiaWrapper.getLoader().getTrackables().activate();

        //raiser.retractFlipper();

        telemetry.update();

        waitForStart();

        //STEP 1: Scan vuforia pattern
        relicRecoveryVuMark = Common.scanVuMark(this, vuforiaWrapper);

        intake.flipOutIntake();

        //STEP 2: Hitting the jewel
        Common.hitJewel(this, armRotator, armExtender, colorSensorWrapper, true);

        sleep(1000);

        // STEP 3: MOVE OFF BALANCING STONE
        this.drivetrain.complexDrive(MecanumDrive.Direction.UP.angle(), 1, 0);
        // 115cm
        double voltage = hardwareMap.voltageSensor.get("Expansion Hub 1").getVoltage();
        sleep(TimeOffsetVoltage.calculateDistance(voltage, 80));

        robot.stopMoving();
        sleep(1000);

        drivetrain.setAngle(imuWrapper, 0);
        sleep(1000);

        //Back is touching balancing stone
        drivetrain.complexDrive(MecanumDrive.Direction.DOWN.angle(), 1, 0);
        sleep(750);

        robot.stopMoving();
        sleep(500);

        //Move back to cryptobox, now with consistent distances.
        drivetrain.complexDrive(MecanumDrive.Direction.UP.angle(), 1, 0);
        this.voltage = hardwareMap.voltageSensor.get("Expansion Hub 1").getVoltage();
        sleep((long) TimeOffsetVoltage.calculateDistance(voltage, 40));

        robot.stopMoving();
        sleep(1000);

        //Face cryptobox
        drivetrain.setAngle(imuWrapper, -Math.PI/2);
        sleep(1000);

        //Move forward a bit
        drivetrain.complexDrive(MecanumDrive.Direction.DOWN.angle(),1,0);
        this.voltage = hardwareMap.voltageSensor.get("Expansion Hub 1").getVoltage();
        sleep(TimeOffsetVoltage.calculateDistance(voltage, 5));

        Common.faceCorrectColumn(this, drivetrain, relicRecoveryVuMark, imuWrapper);

        //SCORE
        drivetrain.stopMoving();
        sleep(1000);

        drivetrain.complexDrive(MecanumDrive.Direction.DOWN.angle(), slamIntoWallSpeed, 0);
        sleep(distanceToWall);

        this.armExtender.setPosition(0.79);
        this.armRotator.setPosition(0.309);
        raiser.outtakeGlyph();

        drivetrain.stopMoving();
        sleep(500);
        raiser.retractFlipper();

        telemetry.update();

        sleep(1000);

        this.drivetrain.complexDrive(MecanumDrive.Direction.DOWN.angle(), slamIntoWallSpeed, 0);
        sleep(distanceToWall + 150);
        this.drivetrain.stopMoving();

        sleep(750);
        this.drivetrain.complexDrive(MecanumDrive.Direction.UP.angle(), slamIntoWallSpeed, 1);
        sleep(350);
        this.drivetrain.stopMoving();

        // TODO: REMOVE IF NOT WORK THIS IS PSEUDO CODE
        this.attemptToGetMultiBlock();
        sleep(500);
        Common.faceCorrectColumn(this, drivetrain, relicRecoveryVuMark, imuWrapper);
        this.drivetrain.complexDrive(MecanumDrive.Direction.DOWN.angle(), 1, 0);
        sleep(350);
        this.drivetrain.stopMoving();
        // pull out
        this.drivetrain.complexDrive(MecanumDrive.Direction.UP.angle(), 1, 0);
        sleep(300);
        this.drivetrain.stopMoving();
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
        sleep(TimeOffsetVoltage.calculateDistance(hardwareMap.voltageSensor.get("Expansion Hub 1").getVoltage(), 70));
        drivetrain.stopMoving();
    }
}
