package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.LightSensor;

/**
 * Created by mikab_000 on 11/27/2016.
 */

@Autonomous(name="BIL: Light Sensor Test", group="BIL")
public class BILLightSensorTest extends LinearOpMode {

    LightSensor lightSensor;

    @Override
    public void runOpMode() throws InterruptedException {
        //button values
        boolean prevButtonDown = false;
        boolean buttonDown = false;

        //led on or off
        boolean ledOn = true;

        //gets light sensor
        lightSensor = hardwareMap.lightSensor.get("lightSensor");

        //sets the led to off
        lightSensor.enableLed(ledOn);

        //waits
        waitForStart();

        while(opModeIsActive()) {
            //gets gamepad1 x button
            buttonDown = gamepad1.x;

            //if button just pressed
            if(buttonDown && !prevButtonDown) {
                //toggle the led
                ledOn = !ledOn;

                //set the led
                lightSensor.enableLed(ledOn);
            }

            //takes the current value and stuffs it into the previous value
            prevButtonDown = buttonDown;

            //telemetry
            telemetry.addData("LED", ledOn ? "On" : "Off");
            telemetry.addData("Raw", lightSensor.getRawLightDetected());
            telemetry.addData("Normal", lightSensor.getLightDetected());

            telemetry.update();
            idle(); // Always call idle() at the bottom of your while(opModeIsActive()) loop
        }
    }
}
