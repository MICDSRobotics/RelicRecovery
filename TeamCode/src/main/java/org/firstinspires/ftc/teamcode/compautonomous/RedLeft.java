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
    private ComplexRaiser complexRaiser;
    private FlipperIntake flipperIntake;
    private MecanumDrive drivetrain;
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
        this.complexRaiser = new ComplexRaiser(hardwareMap);
        this.flipperIntake = new FlipperIntake(hardwareMap);
        imuWrapper = new IMUWrapper(hardwareMap);
        vuforiaWrapper = new VuforiaWrapper(hardwareMap);

        //Assuming other hardware not yet on the robot
        armRotator = hardwareMap.servo.get("armRotator");
        armExtender = hardwareMap.servo.get("armExtender");

        armRotator.scaleRange(0.158, 0.7);
        armExtender.scaleRange(0.16, 0.95);

        armExtender.setPosition(1.0);
        armRotator.setPosition(1.0);

        colorSensorWrapper = new ColorSensorWrapper(hardwareMap);

        vuforiaWrapper.getLoader().getTrackables().activate();

        waitForStart();

        //STEP 1: Scan vuforia pattern
        relicRecoveryVuMark = RelicRecoveryVuMark.from(vuforiaWrapper.getLoader().getRelicTemplate());

        if (relicRecoveryVuMark != RelicRecoveryVuMark.UNKNOWN) {
            telemetry.addData("VuMark Column", relicRecoveryVuMark.name());
        } else {
            telemetry.addData("VuMark Column", "borked");
        }
        telemetry.update();

        //STEP 2: Hitting the jewel
        armRotator.setPosition(0.5);
        sleep(1000);
        armExtender.setPosition(0); //servo in 'out' position
        sleep(1500);

        telemetry.addData("Color Sensor", "R: %f \nB: %f ", colorSensorWrapper.getRGBValues()[0], colorSensorWrapper.getRGBValues()[2]);

        //Checks that blue jewel is closer towards the cryptoboxes (assuming color sensor is facing forward
        if (Math.abs(colorSensorWrapper.getRGBValues()[2] - colorSensorWrapper.getRGBValues()[0]) < 30) {
            telemetry.addData("Jewels", "Too close.");
        } else if (colorSensorWrapper.getRGBValues()[2] > colorSensorWrapper.getRGBValues()[0]) {
            armRotator.setPosition(1);
            telemetry.addData("Jewels", "Blue Team!");
        } else {
            armRotator.setPosition(0);
            telemetry.addData("Jewels", "Red Team!");
        }
        telemetry.update();
        sleep(500);

        armExtender.setPosition(1);
        armRotator.setPosition(0.5);

        sleep(500);

        this.complexRaiser.raiseUp();
        sleep(500);
        this.complexRaiser.stop();

        // Move backwards off balancing stone
        this.drivetrain.complexDrive(MecanumDrive.Direction.DOWN.angle(), 1, 0); // move backwards
        // 160cm
        double voltage = hardwareMap.voltageSensor.get("Expansion Hub 1").getVoltage();
        sleep((long)TimeOffsetVoltage.calculateDistance(voltage, 160));
        this.drivetrain.stopMoving();
        sleep(100);

        //Turn 90 degrees to face cryptobox
        drivetrain.setAngle(imuWrapper, -(float)Math.PI/2);
        sleep(500);

        // Move in front of correct cryptobox column
        switch (relicRecoveryVuMark) {
            case LEFT: telemetry.addData("Column", "Putting it in the left");
                drivetrain.complexDrive(MecanumDrive.Direction.LEFT.angle(), 0.4, 0);
                sleep((long)(1100 + sideShort));
                break;
            case CENTER: telemetry.addData("Column", "Putting it in the center");
                break;
            case RIGHT: telemetry.addData("Column", "Putting it in the right");
                drivetrain.complexDrive(MecanumDrive.Direction.RIGHT.angle(), 0.4, 0);
                sleep((long)(1100 + sideShort));
                break;
            default:
                break;
        }

        telemetry.update();

        this.complexRaiser.getX().spitOutGlyph();
        drivetrain.stopMoving();
        sleep(500);

        drivetrain.complexDrive(MecanumDrive.Direction.UP.angle(), slamIntoWallSpeed, 0);
        sleep(200);

        drivetrain.stopMoving();
        sleep(200);

        wiggle();
        wiggle();

        // PULL OUT (Once)
        this.drivetrain.complexDrive(MecanumDrive.Direction.UP.angle(), 1, 0);
        sleep(150);
        this.drivetrain.stopMoving();

        telemetry.update();

        sleep(500);

        //Move back to center of cryptobox tape (if necessary)
        // START
        switch (relicRecoveryVuMark) {
            case LEFT:
                drivetrain.complexDrive(MecanumDrive.Direction.RIGHT.angle(), 0.4, 0);
                sleep((long)(1100 + sideShort));
                break;
            case CENTER:
                break;
            case RIGHT:
                drivetrain.complexDrive(MecanumDrive.Direction.LEFT.angle(), 0.4, 0);
                sleep((long)(1100 + sideShort));
                break;
            default:
                break;
        }

        this.complexRaiser.lower();
        sleep(450);
        this.complexRaiser.stop();

        this.attemptToGetMultiBlock();

        this.drivetrain.stopMoving();
        sleep(200);

        //REPEAT PUTTING BLOCK IN THE THING
        //Turn 90 degrees to face cryptobox
        drivetrain.setAngle(imuWrapper, (float)Math.PI/2);
        sleep(500);

        // Move in front of correct cryptobox column
        switch (relicRecoveryVuMark) {
            case LEFT: telemetry.addData("Column", "Putting it in the left");
                drivetrain.complexDrive(MecanumDrive.Direction.LEFT.angle(), 0.4, 0);
                sleep((long)(1100 + sideShort));
                break;
            case CENTER: telemetry.addData("Column", "Putting it in the center");
                break;
            case RIGHT: telemetry.addData("Column", "Putting it in the right");
                drivetrain.complexDrive(MecanumDrive.Direction.RIGHT.angle(), 0.4, 0);
                sleep((long)(1100 + sideShort));
                break;
            default:
                break;
        }

        telemetry.update();

        drivetrain.complexDrive(MecanumDrive.Direction.UP.angle(), slamIntoWallSpeed, 0);
        sleep(200);

        this.complexRaiser.getX().spitOutGlyph();
        drivetrain.stopMoving();
        sleep(500);
        this.complexRaiser.getX().retractOuttake();

        // PULL OUT (Once)
        this.drivetrain.complexDrive(MecanumDrive.Direction.DOWN.angle(), 1, 0);
        sleep(150);
        this.drivetrain.stopMoving();

    }

    public void wiggle(){
        drivetrain.complexDrive(MecanumDrive.Direction.UPLEFT.angle(), 0.75, 0);
        sleep(150);
        drivetrain.complexDrive(MecanumDrive.Direction.UP.angle(), 0.75, 0);
        sleep(150);
        drivetrain.complexDrive(MecanumDrive.Direction.UPRIGHT.angle(), 0.75, 0);
        sleep(150);
    }

    public void attemptToGetMultiBlock() {
        // Face glyph horde using the gyro
        this.drivetrain.setAngle(this.imuWrapper, (float)(-Math.PI/2));
        // briefly ram into the block pile
        this.drivetrain.complexDrive(MecanumDrive.Direction.UP.angle(), 1, 0);
        sleep(1000);
        this.drivetrain.stopMoving();
        sleep(500);

        // Attempt to pick up a block
        this.flipperIntake.startIntake();
        sleep(500);
        this.complexRaiser.getX().spitOutGlyph();
        sleep(500);
        drivetrain.complexDrive(MecanumDrive.Direction.DOWN.angle(), 0.5, 0);
        sleep(100);
        drivetrain.stopMoving();

        //Raise up
        this.complexRaiser.raiseUp();
        sleep(600);
        this.complexRaiser.stop();
        sleep(200);
        // Move back towards cryptobox
        this.drivetrain.complexDrive(MecanumDrive.Direction.DOWN.angle(), 1, 0);
        sleep(TimeOffsetVoltage.calculateDistance(hardwareMap.voltageSensor.get("Expansion Hub 1").getVoltage(), 70));
        drivetrain.stopMoving();
    }
}
