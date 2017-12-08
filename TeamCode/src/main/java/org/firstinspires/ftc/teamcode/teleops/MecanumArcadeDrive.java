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

    Servo armX;
    Servo armY;
    double x = 0.0;
    double y = 0.0;

    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        robot = new Robot(hardwareMap);
        game1 = new ControllerWrapper(gamepad1);
        drivetrain = (MecanumDrive) robot.getDrivetrain();

        armX = hardwareMap.servo.get("armX");
        armY = hardwareMap.servo.get("armY");
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

        if (gamepad1.dpad_left) {
            this.armX.setPosition(this.x-=0.01);
        }
        else if (gamepad1.dpad_right) {
            this.armX.setPosition(this.x+=0.01);
        }
        else if (gamepad1.dpad_up) {
            this.armY.setPosition(this.y-=0.01);
        }
        else if (gamepad1.dpad_down) {
            this.armY.setPosition(this.y+=0.01);
        }

        telemetry.addData("Main1", drivetrain.getmajorDiagonal().getMotor1().getPower());
        telemetry.addData("Minor1", drivetrain.getMinorDiagonal().getMotor1().getPower());
        telemetry.addData("Minor2", drivetrain.getMinorDiagonal().getMotor2().getPower());
        telemetry.addData("Main2", drivetrain.getmajorDiagonal().getMotor2().getPower());
        telemetry.addData("X", this.armX.getPosition());
        telemetry.addData("Y", this.armY.getPosition());
    }

    // Code to run ONCE after the driver hits STOP
    @Override
    public void stop() {

    }

}
