package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robotplus.hardware.IMUWrapper;
import org.firstinspires.ftc.teamcode.robotplus.hardware.MecanumDrive;
import org.firstinspires.ftc.teamcode.robotplus.hardware.Robot;
import org.firstinspires.ftc.teamcode.robotplus.hardware.TankDrive;

/**
 * Created by BAbel on 11/3/2017.
 */

@TeleOp (name = "IMU Position Testing", group = "Tester")
public class PositionTesting extends OpMode {

    private Robot robot;
    private IMUWrapper imuWrapper;

    @Override
    public void init() {
        robot = new Robot(hardwareMap);
    }

    @Override
    public void loop() {
        if(robot.getDrivetrain() instanceof MecanumDrive){
            ((MecanumDrive) robot.getDrivetrain()).complexDrive(gamepad1, telemetry);
        } else if (robot.getDrivetrain() instanceof TankDrive){
            ((TankDrive) robot.getDrivetrain()).getLeftMotors().setPowers(gamepad1.left_stick_y);
            ((TankDrive) robot.getDrivetrain()).getRightMotors().setPowers(gamepad1.right_stick_y);
        }

        telemetry.addData("X Position:", imuWrapper.getPosition().x);
        telemetry.addData("Y Position:", imuWrapper.getPosition().y);
        telemetry.addData("Z Position:", imuWrapper.getPosition().z);
    }

    @Override
    public void stop() {
        robot.stopMoving();
    }
}
