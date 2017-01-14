package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import java.util.List;

/**
 * Created on 11/12/2016  by Mika.
 */
@Autonomous(name="BIL: Red Beacons", group="BIL")
public class BILVuforiaRedBeacons extends BILAutonomousCommon {

    VuforiaLocalizer vuforia;
    BILVuforiaCommon helper = new BILVuforiaCommon();

    @Override public void runOpMode() throws InterruptedException{
        this.vuforia = helper.initVuforia(false, 4);
        VuforiaTrackables targets = helper.loadTargets("FTC_2016-17", "Wheels", "Tools", "Legos", "Gears");

        robot.init(hardwareMap);

        robot.lightSensor.enableLed(true);
        robot.colorSensor.enableLed(false);

        Thread.sleep(50);

        darkFloorValue = robot.lightSensor.getLightDetected();
        while(!isStarted()) {
            darkFloorValue = (darkFloorValue + robot.lightSensor.getLightDetected())/2;

            //continuously calibrate gyro to keep the heading as accurate as possible
            if(!robot.gyroSensor.isCalibrating()){
                robot.gyroSensor.calibrate();
            }

            idle();
        }

        //wait for the gyro to finish calibrating if it is
        while(robot.gyroSensor.isCalibrating()){
            idle();
        }

        waitForStart();


        driveUntilLineOrDistance(0.5, 6.5);
        //turn 45 degrees the first 40 degrees at 0.5 speed, and to not overshoot the last 5 degrees would be 0.1 speed
        turnDegrees(0.5, -40);
        turnDegrees(0.1, -5);

//        diagonalUntilLineOrTime(3000);

        targets.activate(); //activate the tracking of the image targets once the opmode starts

        List<VuforiaTrackable> redTrackablesList = helper.returnRedTargets(targets);

        VuforiaTrackable toolsTarget = redTrackablesList.get(0);
        VuforiaTrackable gearsTarget = redTrackablesList.get(1);

        // Strafe one way then another looking for the white line
        findWhiteLine();

        // Once it's in front of the image, adjust and move forward until exactly in front of image, within x mm.
        moveToImage(gearsTarget, helper);

        telemetry.addData("Red:", robot.colorSensor.red());
        telemetry.addData("Blue:", robot.colorSensor.blue());
        telemetry.update();
        if(robot.colorSensor.red() > robot.colorSensor.blue()){ //left side red
            robot.pusher.setPosition(robot.pusherLeft);
        } else if(robot.colorSensor.blue() > robot.colorSensor.red()) { //right side is red
            robot.pusher.setPosition(robot.pusherRight);
        }
        driveDistance(0.2, 0.25);

        // Drive backwards 400 ms
        driveByTime(-0.5, 400);
        robot.pusher.setPosition(robot.pusherMiddle);

//        strafeUntilLineOrTime(0.5, 2000);

        driveBetweenBeacons(90);

        //find the white line
        findWhiteLine();

        moveToImage(toolsTarget, helper);

        telemetry.addData("Red:", robot.colorSensor.red());
        telemetry.addData("Blue:", robot.colorSensor.blue());
        telemetry.update();
        if(robot.colorSensor.red() > robot.colorSensor.blue()){ //left side red
            robot.pusher.setPosition(robot.pusherLeft);
        } else if(robot.colorSensor.blue() > robot.colorSensor.red()) { //right side is red
            robot.pusher.setPosition(robot.pusherRight);
        }
        driveDistance(0.2, 0.25);

        driveByTime(-0.5, 400);
        robot.pusher.setPosition(robot.pusherMiddle);
        setAllDriveMotors(0);
    }
}
