package org.firstinspires.ftc.teamcode.autonomous.gyroautos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.teamcode.compautonomous.Settings;
import org.firstinspires.ftc.teamcode.robotplus.autonomous.VuforiaWrapper;
import org.firstinspires.ftc.teamcode.robotplus.hardware.ColorSensorWrapper;
import org.firstinspires.ftc.teamcode.robotplus.hardware.GrabberPrimer;
import org.firstinspires.ftc.teamcode.robotplus.hardware.IMUWrapper;
import org.firstinspires.ftc.teamcode.robotplus.hardware.MecanumDrive;
import org.firstinspires.ftc.teamcode.robotplus.hardware.Robot;

/**
 * Blue Right Scenario
 * @author Alex M, Blake A
 * @since 12/30/17
 */

@Autonomous(name="Gyro Blue Right", group="gyro")
@Disabled
public class GyroBR extends LinearOpMode implements Settings{

    private Robot robot;
    private DcMotor raiser;
    private Servo grabber;
    private MecanumDrive drivetrain;
    private IMUWrapper imuWrapper;
    private VuforiaWrapper vuforiaWrapper;

    private Servo armExtender;
    private Servo armRotator;
    private ColorSensorWrapper colorSensorWrapper;
    private GrabberPrimer grabberPrimer;


    private RelicRecoveryVuMark relicRecoveryVuMark;

    @Override
    public void runOpMode() throws InterruptedException {

        //Initialize hardware
        robot = new Robot(hardwareMap);
        drivetrain = (MecanumDrive) robot.getDrivetrain();
        raiser = hardwareMap.dcMotor.get("raiser");
        grabber = hardwareMap.servo.get("grabber");
        imuWrapper = new IMUWrapper(hardwareMap);
        vuforiaWrapper = new VuforiaWrapper(hardwareMap);
        grabberPrimer = new GrabberPrimer(this.grabber);


        //Assuming other hardware not yet on the robot
        armRotator = hardwareMap.servo.get("armRotator");
        armExtender = hardwareMap.servo.get("armExtender");

        armRotator.scaleRange(0.1, 0.9);
        armExtender.scaleRange(0.16, 0.85);

        armExtender.setPosition(1.0);
        armRotator.setPosition(0.5);

        grabberPrimer.initSystem();

        colorSensorWrapper = new ColorSensorWrapper(hardwareMap);

        vuforiaWrapper.getLoader().getTrackables().activate();

        telemetry.addData("Grabber", grabber.getPosition());
        telemetry.update();

        waitForStart();

        //grab the block before moving off balancing stone
        grabberPrimer.grab();

        //STEP 1: Scan vuforia pattern
        relicRecoveryVuMark = RelicRecoveryVuMark.from(vuforiaWrapper.getLoader().getRelicTemplate());

        if (relicRecoveryVuMark != RelicRecoveryVuMark.UNKNOWN) {
            telemetry.addData("VuMark Column", relicRecoveryVuMark.name());
        } else {
            telemetry.addData("VuMark Column", "borked");
        }
        telemetry.update();

        //STEP 2: Hitting the jewel
        armExtender.setPosition(0); //servo in 'out' position

        sleep(2000);

        telemetry.addData("Color Sensor", "R: %f \nB: %f ", colorSensorWrapper.getRGBValues()[0], colorSensorWrapper.getRGBValues()[2]);
        //Checks that blue jewel is closer towards the cryptoboxes (assuming color sensor is facing forward
        if (Math.abs(colorSensorWrapper.getRGBValues()[2] - colorSensorWrapper.getRGBValues()[0]) < 30) {
            telemetry.addData("Jewels", "Too close.");
        } else if (colorSensorWrapper.getRGBValues()[2] > colorSensorWrapper.getRGBValues()[0]) {
            armRotator.setPosition(0);
            telemetry.addData("Jewels", "Blue Team!");
        } else {
            armRotator.setPosition(1);
            telemetry.addData("Jewels", "Red Team!");
        }
        telemetry.update();

        sleep(1000);

        raiser.setPower(1);
        sleep(150);
        raiser.setPower(0);

        armExtender.setPosition(1);
        armRotator.setPosition(0.5);

        sleep(1000);

        //imuWrapper.getIMU().initialize(imuWrapper.getIMU().getParameters());

        drivetrain.complexDrive(MecanumDrive.Direction.UP.angle(), 1, 0);
        sleep(firstStretch);

        robot.stopMoving();
        sleep(1000);

        //telemetry.addData("FirstAngle", imuWrapper.getOrientation().firstAngle);
        //telemetry.update();

        setAngle(imuWrapper, drivetrain, (float)Math.PI/2);
        sleep(1000);

        switch (relicRecoveryVuMark) {
            case LEFT: telemetry.addData("Column", "Putting it in the left");
                drivetrain.complexDrive(MecanumDrive.Direction.LEFT.angle(), 0.75, 0);
                sleep((long)(sideShort));
                break;
            case CENTER: telemetry.addData("Column", "Putting it in the center");
                break;
            case RIGHT: telemetry.addData("Column", "Putting it in the right");
                drivetrain.complexDrive(MecanumDrive.Direction.RIGHT.angle(), 0.75, 0);
                sleep((long)(sideShort));
                break;
            default:
                break;
        }

        telemetry.update();

        grabberPrimer.open();
        sleep(1000);

        drivetrain.complexDrive(MecanumDrive.Direction.UP.angle(), slamIntoWallSpeed, 0);
        sleep(200);

        drivetrain.stopMoving();
        sleep(200);

        wiggle();
        wiggle();

        // PULL OUT
        this.drivetrain.complexDrive(MecanumDrive.Direction.DOWN.angle(), 1, 0);
        sleep(100);
        this.drivetrain.stopMoving();
        sleep(100);
        switch (relicRecoveryVuMark) {
            case LEFT: this.drivetrain.complexDrive(MecanumDrive.Direction.RIGHT.angle(), 1, 0);
                break;
            case CENTER: this.drivetrain.complexDrive(MecanumDrive.Direction.DOWN.angle(), 1, 0);
                break;
            case RIGHT: this.drivetrain.complexDrive(MecanumDrive.Direction.LEFT.angle(), 1, 0);
                break;
            default: this.drivetrain.complexDrive(MecanumDrive.Direction.DOWN.angle(), 1, 0);
                break;
        }
        sleep(100);
        this.drivetrain.stopMoving();
        sleep(100);
        this.drivetrain.complexDrive(MecanumDrive.Direction.DOWN.angle(), 1, 0);
        sleep(100);
        this.drivetrain.stopMoving();
        sleep(100);

        telemetry.update();

        sleep(1000);
    }

    public void wiggle(){
        drivetrain.complexDrive(MecanumDrive.Direction.UPLEFT.angle() + 0.5, 0.75, 0);
        sleep(150);
        drivetrain.complexDrive(MecanumDrive.Direction.UP.angle(), 0.75, 0);
        sleep(150);
        drivetrain.complexDrive(MecanumDrive.Direction.UPRIGHT.angle() - 0.5, 0.75, 0);
        sleep(150);
    }

    public void setAngle(IMUWrapper imuWrapper, MecanumDrive drivetrain, float angle){

        float heading = imuWrapper.getOrientation().toAngleUnit(AngleUnit.RADIANS).firstAngle;

        while (!(heading > angle - 0.025 && heading < angle + 0.025 )){
            if (heading > angle) {
                drivetrain.complexDrive(0, 0, 0.5);
            } else {
                drivetrain.complexDrive(0, 0, -0.5);
            }
            heading = imuWrapper.getOrientation().toAngleUnit(AngleUnit.RADIANS).firstAngle;
        }

        drivetrain.stopMoving();

    }

}