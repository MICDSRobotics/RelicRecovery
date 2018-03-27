package org.firstinspires.ftc.teamcode.autonomous;

import android.content.Context;
import android.os.Environment;
import android.os.Handler;
import android.os.Looper;
import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Hardware;

import org.firstinspires.ftc.teamcode.robotplus.hardware.MecanumDrive;
import org.firstinspires.ftc.teamcode.robotplus.hardware.Robot;
import org.firstinspires.ftc.teamcode.robotplus.inputtracking.Input;
import org.firstinspires.ftc.teamcode.robotplus.inputtracking.InputWriter;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.IOException;
import java.io.FileOutputStream;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.io.OutputStreamWriter;
import java.util.ArrayList;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a PushBot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Recorder", group="potato") // @Autonomous(...) is the other common choice
public class Recording extends OpMode {

    private ElapsedTime runtime = new ElapsedTime();
    //private Robot robot;

    //private MecanumDrive drivetrain;

    private ArrayList<Input> inputs;
    private File directory;
    private File file;

    private String filename = "PLS WORK.json";

    FileOutputStream outputStream;

    /*
    Looper looper;

    public Looper getLooper() {
        return looper;
    }
    */

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {

        //robot = new Robot(hardwareMap);
        //drivetrain = (MecanumDrive) robot.getDrivetrain();

        if (Environment.MEDIA_MOUNTED.equals(Environment.getExternalStorageState())) {
            Log.d("INPUT RECORDER", "good to go homie");
        } else {
            Log.d("INPUT RECORDER", "FUCK");
        }

        directory = getStorageDir(hardwareMap.appContext, "aw man");

        Log.d("INPUT RECORDER", directory.getAbsolutePath());

        file = new File(directory, filename);

        Log.d("INPUT RECORDER", file.getAbsolutePath());

        inputs = new ArrayList<Input>();

        telemetry.addData("Status", "Initialized");
        Log.d("INPUT RECORDER", "inited");

    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
        Log.d("INPUT RECORDER", "started");
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        telemetry.addData("Status", "Running: " + runtime.toString());

        //drivetrain.complexDrive(gamepad1, telemetry);

        inputs.add(new Input(gamepad1, runtime.time()));

        Log.v("INPUT RECORDER", gamepad1.toString());

    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {

        Log.d("INPUT RECORDER", "stopping");

        try {
            outputStream = hardwareMap.appContext.openFileOutput(filename, Context.MODE_WORLD_READABLE);
            OutputStreamWriter outputStreamWriter = new OutputStreamWriter(outputStream);

            //InputWriter writer = new InputWriter();
            //writer.writeJson(outputStream, inputs);

            outputStreamWriter.write("Hello World!");
            outputStreamWriter.close();
            telemetry.addData("Output", outputStream);
            Log.d("INPUT RECORDER", "wrote");
        } catch (IOException error){
            error.printStackTrace();
            Log.d("INPUT RECORDER", "didn't wrote");
        }

        //robot.stopMoving();

    }


    public File getStorageDir(Context context, String fileName) {
        // Get the directory for the app's private pictures directory.
        File file = new File(Environment.getExternalStoragePublicDirectory(
                Environment.DIRECTORY_DOCUMENTS), fileName);
        if (!file.mkdirs()) {
            Log.e("INPUT RECORDER", "Directory not created");
        }
        return file;
    }

    private String readFromFile(Context context) {

        String ret = "";

        try {
            InputStream inputStream = context.openFileInput("config.txt");

            if ( inputStream != null ) {
                InputStreamReader inputStreamReader = new InputStreamReader(inputStream);
                BufferedReader bufferedReader = new BufferedReader(inputStreamReader);
                String receiveString = "";
                StringBuilder stringBuilder = new StringBuilder();

                while ( (receiveString = bufferedReader.readLine()) != null ) {
                    stringBuilder.append(receiveString);
                }

                inputStream.close();
                ret = stringBuilder.toString();
            }
        }
        catch (FileNotFoundException e) {
            Log.e("login activity", "File not found: " + e.toString());
        } catch (IOException e) {
            Log.e("login activity", "Can not read file: " + e.toString());
        }

        return ret;
    }


}
