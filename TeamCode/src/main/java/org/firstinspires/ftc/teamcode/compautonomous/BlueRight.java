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

        armExtender.setPosition(1.0);
        armRotator.setPosition(1.0);

        colorSensorWrapper = new ColorSensorWrapper(hardwareMap);

        vuforiaWrapper.getLoader().getTrackables().activate();

        raiser.retractFlipper();

        telemetry.update();

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
        armExtender.setPosition(0.8);
        sleep(1000);
        armExtender.setPosition(0); //servo in 'out' position
        sleep(1500);

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

        armExtender.setPosition(0.8);
        armRotator.setPosition(0.5);

        sleep(1000);

        // move backwards and slam into the wall
        this.intake.flipInIntake();
        this.drivetrain.complexDrive(MecanumDrive.Direction.UP.angle(), 1, 0); // move backwards
        // 78cm
        this.voltage = hardwareMap.voltageSensor.get("Expansion Hub 1").getVoltage();
        // 170
        sleep((long) TimeOffsetVoltage.calculateDistance(voltage, 170));
        this.drivetrain.stopMoving();
        this.intake.stopIntake();
        sleep(100);

        robot.stopMoving();
        sleep(1000);

        //Face cryptobox
        drivetrain.setAngle(imuWrapper, -Math.PI/2);
        sleep(1000);

        //Lower raiser a bit
        this.raiser.lower();
        sleep(500);
        this.raiser.stop();

        moveToCorrectColumn();

        telemetry.update();

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
        this.moveToCorrectColumn();
        this.drivetrain.complexDrive(MecanumDrive.Direction.DOWN.angle(), 1, 0);
        sleep(350);
        this.drivetrain.stopMoving();
        // pull out
        this.drivetrain.complexDrive(MecanumDrive.Direction.UP.angle(), 1, 0);
        sleep(300);
        this.drivetrain.stopMoving();
    }

    //Method to help guard against glyph getting stuck between columns
    public void wiggle(){
        drivetrain.complexDrive(MecanumDrive.Direction.DOWNLEFT.angle(), 0.75, 0);
        sleep(150);
        drivetrain.complexDrive(MecanumDrive.Direction.DOWN.angle(), 0.75, 0);
        sleep(150);
        drivetrain.complexDrive(MecanumDrive.Direction.DOWNRIGHT.angle(), 0.75, 0);
        sleep(150);
    }

    public void moveToCorrectColumn(){
        switch (relicRecoveryVuMark) {
            case LEFT:
                telemetry.addData("Column", "Putting it in the left");
                //drivetrain.complexDrive(MecanumDrive.Direction.RIGHT.angle(), 0.4, 0);
                //sleep((long) (1100 + sideShort));
                drivetrain.setAngle(imuWrapper, -Math.PI * 11 / 12);
                break;
            case CENTER:
                telemetry.addData("Column", "Putting it in the center");
                break;
            case RIGHT:
                telemetry.addData("Column", "Putting it in the right");
                //drivetrain.complexDrive(MecanumDrive.Direction.RIGHT.angle(), 0.4, 0);
                //sleep((long) (1100 + sideShort));
                drivetrain.setAngle(imuWrapper, Math.PI * 11 / 12);
                break;
            default:
                break;
        }
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
