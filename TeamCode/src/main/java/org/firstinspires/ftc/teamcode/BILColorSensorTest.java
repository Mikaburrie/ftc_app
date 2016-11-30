package org.firstinspires.ftc.teamcode;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;

/**
 * Created by mikab_000 on 11/29/2016.
 */
@Autonomous(name="BIL: Color Sensor Test", group="BIL")
public class BILColorSensorTest extends LinearOpMode{

    ColorSensor colorSensor;

    @Override
    public void runOpMode() throws InterruptedException {

        // hsvValues is an array that will hold the hue, saturation, and value information.
        float hsvValues[] = {0F,0F,0F};

        // values is a reference to the hsvValues array.
        final float values[] = hsvValues;

        colorSensor = hardwareMap.colorSensor.get("colorSensor");

        // wait for the start button to be pressed.
        waitForStart();

        // loop and read the RGB data.
        // Note we use opModeIsActive() as our loop condition because it is an interruptible method.
        while (opModeIsActive())  {
            // convert the RGB values to HSV values.
            Color.RGBToHSV(colorSensor.red(), colorSensor.green(), colorSensor.blue(), hsvValues);
            // send the info back to driver station using telemetry function.
            telemetry.addData("Clear", colorSensor.alpha());
            telemetry.addData("Red", colorSensor.red());
            telemetry.addData("Green", colorSensor.green());
            telemetry.addData("Blue", colorSensor.blue());
            telemetry.addData("Hue", hsvValues[0]);

            telemetry.update();
            idle(); // Always call idle() at the bottom of your while(opModeIsActive()) loop
        }
    }
}
