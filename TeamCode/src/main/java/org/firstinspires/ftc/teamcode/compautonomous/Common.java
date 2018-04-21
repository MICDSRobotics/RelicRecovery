package org.firstinspires.ftc.teamcode.compautonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.teamcode.robotplus.autonomous.TimeOffsetVoltage;
import org.firstinspires.ftc.teamcode.robotplus.autonomous.VuforiaWrapper;
import org.firstinspires.ftc.teamcode.robotplus.hardware.ColorSensorWrapper;
import org.firstinspires.ftc.teamcode.robotplus.hardware.ComplexRaiser;
import org.firstinspires.ftc.teamcode.robotplus.hardware.FlipperIntake;
import org.firstinspires.ftc.teamcode.robotplus.hardware.IMUAccelerationIntegrator;
import org.firstinspires.ftc.teamcode.robotplus.hardware.IMUWrapper;
import org.firstinspires.ftc.teamcode.robotplus.hardware.MecanumDrive;

public class Common {

    public static RelicRecoveryVuMark scanVuMark(LinearOpMode lop, VuforiaWrapper vuforiaWrapper){
        RelicRecoveryVuMark mark = RelicRecoveryVuMark.from(vuforiaWrapper.getLoader().getRelicTemplate());

        if (mark != RelicRecoveryVuMark.UNKNOWN) {
            lop.telemetry.addData("VuMark Column", mark.name());
        } else {
            lop.telemetry.addData("VuMark Column", "borked");
        }
        lop.telemetry.update();

        return mark;
    }

    public static void hitJewel(LinearOpMode lop, Servo armRotator, Servo armExtender,
                                ColorSensorWrapper colorSensorWrapper, boolean isBlueTeam){
        armExtender.setPosition(0.74);
        lop.sleep(1000);
        armRotator.setPosition(0.849);
        lop.sleep(1000);
        armExtender.setPosition(0.03); //servo in 'out' position
        lop.sleep(1500);

        lop.telemetry.addData("Color Sensor", "R: %f \nB: %f ", colorSensorWrapper.getRGBValues()[0], colorSensorWrapper.getRGBValues()[2]);
        //Checks that blue jewel is closer towards the cryptoboxes (assuming color sensor is facing forward)
        if (Math.abs(colorSensorWrapper.getRGBValues()[2] - colorSensorWrapper.getRGBValues()[0]) < 30) {
            lop.telemetry.addData("Jewels", "Too close.");
        } else if (colorSensorWrapper.getRGBValues()[2] > colorSensorWrapper.getRGBValues()[0]) {
            if(isBlueTeam) {
                armRotator.setPosition(1);
                lop.telemetry.addData("Jewels", "Blue Team, hitting off the back!");
            } else {
                armRotator.setPosition(0);
                lop.telemetry.addData("Jewels", "Red Team, hitting off the front!");
            }
        } else {
            if(isBlueTeam) {
                armRotator.setPosition(0);
                lop.telemetry.addData("Jewels", "Blue Team, hitting off the front!");
            } else {
                armRotator.setPosition(1);
                lop.telemetry.addData("Jewels", "Blue Team, hitting off the back!");
            }
        }
        lop.telemetry.update();

        lop.sleep(1000);

        armExtender.setPosition(0.85);
        lop.sleep(500);
        armRotator.setPosition(0.75);
    }

    public static void faceCorrectColumn(LinearOpMode lop, MecanumDrive drivetrain,
                                         RelicRecoveryVuMark vuMark, IMUWrapper imuWrapper){
        switch (vuMark) {
            case LEFT:
                lop.telemetry.addData("Column", "Putting it in the left");

                //drivetrain.complexDrive(MecanumDrive.Direction.RIGHT.angle(), 0.4, 0);
                //sleep((long) (1100 + sideShort));

                //Rotates instead of moving to the side
                drivetrain.setAngle(imuWrapper, -Math.PI * 11 / 12);
                break;
            case CENTER:
                lop.telemetry.addData("Column", "Putting it in the center");
                break;
            case RIGHT:
                lop.telemetry.addData("Column", "Putting it in the right");
                //drivetrain.complexDrive(MecanumDrive.Direction.RIGHT.angle(), 0.4, 0);
                //sleep((long) (1100 + sideShort));

                //Rotates instead of moving to the side
                drivetrain.setAngle(imuWrapper, Math.PI * 11 / 12);
                break;
            default:
                break;
        }

        lop.telemetry.update();
    }

    public static void scoreInColumn(LinearOpMode lop, MecanumDrive dt, ComplexRaiser raiser){
        raiser.outtakeGlyph();
        lop.sleep(3500);
        dt.complexDrive(MecanumDrive.Direction.DOWN.angle(), Settings.slamIntoWallSpeed, 0);
        lop.sleep(Settings.distanceToWall);
        dt.stopMoving();
        lop.sleep(250);

        wiggle(lop, dt);

        dt.complexDrive(MecanumDrive.Direction.DOWN.angle(), 1, 0);
        lop.sleep(Settings.distanceToWall + 150);
        dt.stopMoving();
    }

    //Method to help guard against glyph getting stuck between columns
    public static void wiggle(LinearOpMode lop, MecanumDrive dt){
        dt.complexDrive(MecanumDrive.Direction.DOWNLEFT.angle(), 0.75, 0);
        lop.sleep(150);
        dt.complexDrive(MecanumDrive.Direction.DOWN.angle(), 0.75, 0);
        lop.sleep(150);
        dt.complexDrive(MecanumDrive.Direction.DOWNRIGHT.angle(), 0.75, 0);
        lop.sleep(150);
    }

    public static void backWiggle(LinearOpMode lop, MecanumDrive dt){
        dt.complexDrive(MecanumDrive.Direction.UPLEFT.angle(), 0.75, 0);
        lop.sleep(150);
        dt.complexDrive(MecanumDrive.Direction.UP.angle(), 0.75, 0);
        lop.sleep(150);
        dt.complexDrive(MecanumDrive.Direction.UPRIGHT.angle(), 0.75, 0);
        lop.sleep(150);
    }

    public static void attemptToGetMultiBlock(LinearOpMode lop, MecanumDrive drive, IMUWrapper imu, FlipperIntake intake, ComplexRaiser raiser, HardwareMap hardwareMap) {
        // Face glyph horde using the gyro
        drive.setAngle(imu, -Math.PI/2);
        // briefly ram into the block pile
        raiser.getX().retractOuttake();
        intake.startIntake();
        drive.complexDrive(MecanumDrive.Direction.UP.angle(), 0.5, 0);
        backWiggle(lop, drive);
        backWiggle(lop, drive);
        lop.sleep(TimeOffsetVoltage.calculateDistance(hardwareMap.voltageSensor.get("Expansion Hub 1").getVoltage(), 50));
        drive.stopMoving();
        lop.sleep(500);

        drive.setAngle(imu, -Math.PI/2);

        // Attempt to pick up a block
        /*intake.startIntake();
        drive.complexDrive(MecanumDrive.Direction.UP.angle(), 1, 0);
        lop.sleep(800);
        drive.stopMoving();
        lop.sleep(100);*/

        // Move back towards cryptobox
        drive.complexDrive(MecanumDrive.Direction.DOWN.angle(), 1, 0);
        //55
        lop.sleep(TimeOffsetVoltage.calculateDistance(hardwareMap.voltageSensor.get("Expansion Hub 1").getVoltage(), 45));
        drive.stopMoving();
        raiser.getX().spitOutGlyph();
        lop.sleep(1500);
        wiggle(lop, drive);
        wiggle(lop, drive);
        drive.complexDrive(MecanumDrive.Direction.UP.angle(), 1, 0);
        lop.sleep(300);
        drive.stopMoving();
    }
}
