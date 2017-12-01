package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robotplus.gamepadwrapper.ControllerWrapper;
import org.firstinspires.ftc.teamcode.robotplus.hardware.MecanumDrive;
import org.firstinspires.ftc.teamcode.robotplus.hardware.Robot;

import java.util.ResourceBundle;

/**
 * @author Blake Abel, Alex Migala
 */
@TeleOp(name="Mecanum Arcade Drive", group="Iterative Opmode")
public class MecanumArcadeDrive extends OpMode {
    private ElapsedTime runtime = new ElapsedTime();

    private Robot robot;
  
    private ControllerWrapper game1;

    private MecanumDrive drivetrain;

    Servo clawleft;
    Servo clawright;

    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        robot = new Robot(hardwareMap);
        game1 = new ControllerWrapper(gamepad1);
        drivetrain = (MecanumDrive) robot.getDrivetrain();
        clawleft = hardwareMap.servo.get("clawleft");
        clawright = hardwareMap.servo.get("clawright");
    }

    // Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
    @Override
    public void init_loop() {

    }

    // Code to run ONCE when the driver hits PLAY
    @Override
    public void start() {
        runtime.reset();
    }

    // Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
    @Override
    public void loop() {
        telemetry.addData("Status", "Running: " + runtime.toString());

        drivetrain.complexDrive(gamepad1, telemetry);

        if (gamepad1.a) {
            this.clawright.setPosition(1);
            this.clawleft.setPosition(1);
        }
        else if (gamepad1.b) {
            this.clawright.setPosition(-1);
            this.clawleft.setPosition(-1);
        }

        telemetry.addData("Main1", drivetrain.getmajorDiagonal().getMotor1().getPower());
        telemetry.addData("Minor1", drivetrain.getMinorDiagonal().getMotor1().getPower());
        telemetry.addData("Minor2", drivetrain.getMinorDiagonal().getMotor2().getPower());
        telemetry.addData("Main2", drivetrain.getmajorDiagonal().getMotor2().getPower());

    }

    // Code to run ONCE after the driver hits STOP
    @Override
    public void stop() {

    }

}
