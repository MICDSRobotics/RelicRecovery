package org.firstinspires.ftc.teamcode.compautonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.teamcode.robotplus.autonomous.VuforiaWrapper;
import org.firstinspires.ftc.teamcode.robotplus.hardware.ColorSensorWrapper;
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

    public static void hitJewel(LinearOpMode lop, Servo armRotator, Servo armExtender, ColorSensorWrapper colorSensorWrapper, boolean isBlueTeam){
        armRotator.setPosition(0.5);
        lop.sleep(2000);
        armExtender.setPosition(0.8);
        lop.sleep(1000);
        armExtender.setPosition(0); //servo in 'out' position
        lop.sleep(1500);

        lop.telemetry.addData("Color Sensor", "R: %f \nB: %f ", colorSensorWrapper.getRGBValues()[0], colorSensorWrapper.getRGBValues()[2]);
        //Checks that blue jewel is closer towards the cryptoboxes (assuming color sensor is facing forward)
        if (Math.abs(colorSensorWrapper.getRGBValues()[2] - colorSensorWrapper.getRGBValues()[0]) < 30) {
            lop.telemetry.addData("Jewels", "Too close.");
        } else if (colorSensorWrapper.getRGBValues()[2] > colorSensorWrapper.getRGBValues()[0]) {
            if(isBlueTeam) {
                armRotator.setPosition(0);
                lop.telemetry.addData("Jewels", "Blue Team, hitting off the back!");
            } else {
                armRotator.setPosition(1);
                lop.telemetry.addData("Jewels", "Red Team, hitting off the front!");
            }
        } else {
            if(isBlueTeam) {
                armRotator.setPosition(1);
                lop.telemetry.addData("Jewels", "Blue Team, hitting off the front!");
            } else {
                armRotator.setPosition(0);
                lop.telemetry.addData("Jewels", "Blue Team, hitting off the back!");
            }
        }
        lop.telemetry.update();

        lop.sleep(1000);

        armExtender.setPosition(0.8);
        armRotator.setPosition(0.5);
    }

    public static void faceCorrectColumn(LinearOpMode lop, MecanumDrive drivetrain, RelicRecoveryVuMark vuMark, IMUWrapper imuWrapper){
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
    }

    public static void scoreInColumn(){

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
}
