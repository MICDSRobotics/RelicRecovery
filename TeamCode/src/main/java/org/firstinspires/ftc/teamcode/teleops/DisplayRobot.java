package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robotplus.gamepadwrapper.Controller;
import org.firstinspires.ftc.teamcode.robotplus.hardware.MecanumDrive;
import org.firstinspires.ftc.teamcode.robotplus.hardware.Robot;

/**
 * @author Blake Abel, Alex Migala
 */
@TeleOp(name="Display Robot", group="Iterative Opmode")
public class DisplayRobot extends OpMode {
    private ElapsedTime runtime = new ElapsedTime();

    private Robot robot;
  
    private Controller game1;

    private MecanumDrive drivetrain;

    Servo armX;
    Servo armY;
    double x = 0.0;
    double y = 0.0;

    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        robot = new Robot(hardwareMap);
        game1 = new Controller(gamepad1);
        drivetrain = (MecanumDrive) robot.getDrivetrain();

        armX = hardwareMap.servo.get("armX");
        armY = hardwareMap.servo.get("armY");
        //this.armY = hardwareMap.get(CRServo.class, "armY");
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
            //this.armY.setPower(-1);
        }
        else if (gamepad1.dpad_down) {
            this.armY.setPosition(this.y+=0.01);
            //this.armY.setPower(1);
        }
        if (gamepad1.x) {
            robot.stopMoving();
            drivetrain.stopMoving();
        }

        telemetry.addData("Main1", drivetrain.getMajorDiagonal().getMotor1().getPower());
        telemetry.addData("Minor1", drivetrain.getMinorDiagonal().getMotor1().getPower());
        telemetry.addData("Minor2", drivetrain.getMinorDiagonal().getMotor2().getPower());
        telemetry.addData("Main2", drivetrain.getMajorDiagonal().getMotor2().getPower());
        telemetry.addData("X", this.armX.getPosition());
        telemetry.addData("Y", this.armY.getPosition());
    }

    // Code to run ONCE after the driver hits STOP
    @Override
    public void stop() {

    }

}
