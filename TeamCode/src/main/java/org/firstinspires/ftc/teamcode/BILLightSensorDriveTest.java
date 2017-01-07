package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * Made by Zoha Peterson on 12/31/16.
 */
@Autonomous(name="BIL: Light Sensor Drive Test", group="BIL")
    public class BILLightSensorDriveTest extends BILAutonomousCommon{

        BILRobotHardware robot = new BILRobotHardware();


        @Override
        public void runOpMode() throws InterruptedException {

            robot.init(hardwareMap);
            robot.lightSensor.enableLed(true);

            double darkFloorValue = robot.lightSensor.getLightDetected();
            while(!isStarted()) {
                darkFloorValue = (darkFloorValue + robot.lightSensor.getLightDetected())/2;

                //continuously calibrate gyro to keep the heading as accurate as possible
                if(!robot.gyroSensor.isCalibrating()){
                    robot.gyroSensor.calibrate();
                }

                idle();
            }

//            waitForStart();
            driveUntilLineOrDistance(0.5, 5);


        }
    }

