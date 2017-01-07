package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import java.util.List;

/**
 * Created by mikab_000 on 11/12/2016.
 */
@Autonomous(name="BIL: Red Beacons", group="BIL")
public class BILVuforiaRedBeacons extends BILAutonomousCommon {

    VuforiaLocalizer vuforia;
    BILVuforiaCommon helper = new BILVuforiaCommon();
    ElapsedTime time = new ElapsedTime();

    @Override public void runOpMode() throws InterruptedException{
        this.vuforia = helper.initVuforia(false, 4);
        VuforiaTrackables targets = helper.loadTargets("FTC_2016-17", "Wheels", "Tools", "Legos", "Gears");

        robot.init(hardwareMap);

        robot.lightSensor.enableLed(true);
        robot.colorSensor.enableLed(false);

        Thread.sleep(50);

        double darkFloorValue = robot.lightSensor.getLightDetected();
        double sideSpeed = 0.5;

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


        driveUntilLineOrDistance(0.5, 6, darkFloorValue);
        //turn 45 degrees the first 40 degrees at 0.5 speed, and to not overshoot the last 5 degrees would be 0.1 speed
        turnDegrees(0.5, -40);
        turnDegrees(0.1, -5);

        targets.activate(); //activate the tracking of the image targets once the opmode starts

        List<VuforiaTrackable> redTrackablesList = helper.returnRedTargets(targets);

        boolean imageSeen = false;
        VuforiaTrackable toolsTarget = redTrackablesList.get(0);
        VuforiaTrackable gearsTarget = redTrackablesList.get(1);

        // Strafe one way then another looking for the white line
        if(robot.lightSensor.getLightDetected() < darkFloorValue + lineColorThreshold) {
            setDriveMotors(sideSpeed, -sideSpeed, -sideSpeed, sideSpeed);
            time.reset();
            while(robot.lightSensor.getLightDetected() < darkFloorValue + lineColorThreshold && time.milliseconds() < 500) {
                idle();
            }
            if(time.milliseconds() < 500){
                setAllDriveMotors(0);
            } else {
                setDriveMotors(-sideSpeed, sideSpeed, sideSpeed, -sideSpeed);
                time.reset();
                while(robot.lightSensor.getLightDetected() < darkFloorValue + lineColorThreshold && time.milliseconds() < 1000) {
                    idle();
                }
            }
            setAllDriveMotors(0);
        }

        // Once it's in front of the image, adjust and move forward until exactly in front of image, within x mm.
        while(!imageSeen){
            VectorF translation = helper.getTargetTranslation(gearsTarget);
            if(translation != null && Math.abs(translation.get(2)) > helper.targetImageDistance) { // 2 = z
                helper.driveToTarget(gearsTarget, robot);
            } else {
                if(translation != null){
                    imageSeen = true;
                } else {
                    telemetry.addData("Gears Target", "not in view");
                    telemetry.update();
                }
            }
            idle();
        }

        telemetry.addData("Red:", robot.colorSensor.red());
        telemetry.addData("Blue:", robot.colorSensor.blue());
        telemetry.update();
        if(robot.colorSensor.red() >= helper.redBeaconColor){ //left side red
            robot.pusher.setPosition(robot.pusherLeft);
        } else if(robot.colorSensor.blue() >= helper.blueBeaconColor) { //right side is red
            robot.pusher.setPosition(robot.pusherRight);
        }
        Thread.sleep(500);
        robot.pusher.setPosition(robot.pusherMiddle);

        // Drive backwards 200 ms
        driveByTime(-0.5, 400);
        //turn 90 degrees the first 85 degrees at 0.5 speed, and to not overshoot the last 5 degrees would be 0.1 speed
        turnDegrees(0.5, 85);
        turnDegrees(0.1, 5);

        setAllDriveMotors(0.5);
        driveUntilLineOrDistance(0.5, 6, darkFloorValue);
        driveDistance(0.5, 0.5); //to top it off

        setAllDriveMotors(0);
        //turn -90 degrees the first 85 degrees at 0.5 speed, and to not overshoot the last 5 degrees would be 0.1 speed
        turnDegrees(0.5, -85);
        turnDegrees(0.1, -5);

        imageSeen = false;
        //push the button
        if(robot.lightSensor.getLightDetected() < darkFloorValue + lineColorThreshold) {
            setDriveMotors(sideSpeed, -sideSpeed, -sideSpeed, sideSpeed);
            time.reset();
            while(robot.lightSensor.getLightDetected() < darkFloorValue + lineColorThreshold && time.milliseconds() < 500) {
                idle();
            }
            if(time.milliseconds() < 500){
                setAllDriveMotors(0);
            } else {
                setDriveMotors(-sideSpeed, sideSpeed, sideSpeed, -sideSpeed);
                time.reset();
                while(robot.lightSensor.getLightDetected() < darkFloorValue + lineColorThreshold && time.milliseconds() < 1000) {
                    idle();
                }
            }
            setAllDriveMotors(0);
        }

        while(!imageSeen){
            VectorF translation = helper.getTargetTranslation(toolsTarget);
            if (translation != null && Math.abs(translation.get(2)) > helper.targetImageDistance) {
                helper.driveToTarget(toolsTarget, robot);
            } else {
                if(translation != null){
                    imageSeen = true;
                }
            }
            idle();
        }

        telemetry.addData("Red:", robot.colorSensor.red());
        telemetry.addData("Blue:", robot.colorSensor.blue());
        telemetry.update();
        if(robot.colorSensor.red() >= helper.redBeaconColor){ //left side red
            robot.pusher.setPosition(robot.pusherLeft);
        } else if(robot.colorSensor.blue() >= helper.blueBeaconColor) { //right side is red
            robot.pusher.setPosition(robot.pusherRight);
        }
        Thread.sleep(500);
        robot.pusher.setPosition(robot.pusherMiddle);

        driveByTime(-0.5, 400);
        setAllDriveMotors(0);
    }
}
