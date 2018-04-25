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
        armRotator.setPosition(1.0);
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

        //STEP 4: Move to Cryptobox
        this.drivetrain.complexDrive(MecanumDrive.Direction.UP.angle(), 1, 0); // move backwards
        voltage = hardwareMap.voltageSensor.get("Expansion Hub 1").getVoltage();
        sleep(TimeOffsetVoltage.calculateDistance(voltage, 47));
        this.drivetrain.stopMoving();
        this.intake.stopIntake();
        sleep(100);

        //Face cryptobox
        drivetrain.setAngle(imuWrapper, -Math.PI/2);
        sleep(1000);

        Common.faceCorrectColumn(this, drivetrain, relicRecoveryVuMark, imuWrapper);

        //SCORE
        this.armExtender.setPosition(0.79);
        this.armRotator.setPosition(0.309);
        Common.scoreInColumn(this, drivetrain, raiser);

        // TODO: REMOVE IF NOT WORK THIS IS PSEUDO CODE
        Common.attemptToGetMultiBlock(this, drivetrain, imuWrapper, intake, raiser, hardwareMap);
    }
}
