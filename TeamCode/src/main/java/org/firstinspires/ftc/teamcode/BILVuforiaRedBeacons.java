package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import java.util.List;

/**
 * Created by mikab_000 on 11/12/2016.
 */
@Autonomous(name="BIL: Red Beacons", group="BIL")
public class BILVuforiaRedBeacons extends LinearOpMode {

    VuforiaLocalizer vuforia;
    BILVuforiaCommon helper = new BILVuforiaCommon();
    BILRobotHardware robot = new BILRobotHardware();
    ElapsedTime time = new ElapsedTime();
    double floorColorWhite = 0.1;

    @Override public void runOpMode() throws InterruptedException{
        this.vuforia = helper.initVuforia(false, 4);
        VuforiaTrackables targets = helper.loadTargets("FTC_2016-17", "Wheels", "Tools", "Legos", "Gears");

        robot.init(hardwareMap);

        robot.lightSensor.enableLed(true);

        Thread.sleep(50);

        double darkFloorValue = robot.lightSensor.getLightDetected();
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


        robot.setAllDriveMotors(0.5);

        while(robot.lightSensor.getLightDetected() < darkFloorValue + floorColorWhite && opModeIsActive()) {
            //wait for robot to run over line
            idle();
        }
        robot.setAllDriveMotors(0);
        robot.turnDegrees(0.5, -45);

        targets.activate(); //activate the tracking of the image targets once the opmode starts

        List<VuforiaTrackable> redTrackablesList = helper.returnRedTargets(targets);

//        boolean seenImage = false;
        boolean doneWithFirstBeacon = false;
        boolean doneWithSecondBeacon = false;
        boolean inFrontOfImage = false;
        VuforiaTrackable toolsTarget = redTrackablesList.get(0);
        VuforiaTrackable gearsTarget = redTrackablesList.get(1);

        while(!doneWithFirstBeacon && opModeIsActive()){
            OpenGLMatrix position = ((VuforiaTrackableDefaultListener) gearsTarget.getListener()).getPose(); //get positions
            if(position != null && !inFrontOfImage){
                VectorF translation = position.getTranslation();
                double xTrans = (double)translation.get(1); //x and y are switched for horizontal phone
//                double yTrans = (double)translation.get(0);
                double zTrans = (double)translation.get(2);

                double degreesToTurn = Math.toDegrees(Math.atan2(zTrans, xTrans)) + 90; //horizontal phone

                if(Math.abs(zTrans) > 250) {
                    double leftSpeed = Range.clip((40 + degreesToTurn * 2)/100, -Math.abs(zTrans)/2000, Math.abs(zTrans)/2000);
                    double rightSpeed = Range.clip((40 - degreesToTurn * 2)/100, -Math.abs(zTrans)/2000, Math.abs(zTrans)/2000);
                    robot.setDriveMotors(leftSpeed, leftSpeed, rightSpeed, rightSpeed);
                } else {
                    inFrontOfImage = true;
                }
            } else if(inFrontOfImage) {
                //push the button
                if(robot.lightSensor.getLightDetected() < darkFloorValue + floorColorWhite) {
                    robot.setDriveMotors(0.5, -0.5, -0.5, 0.5);
                    time.reset();
                    while(robot.lightSensor.getLightDetected() < darkFloorValue + floorColorWhite && time.milliseconds() < 250) {
                        idle();
                    }
                    if(time.milliseconds() >= 250) {
                        robot.setDriveMotors(-0.5, 0.5, 0.5, -0.5);
                        time.reset();
                        while(robot.lightSensor.getLightDetected() < darkFloorValue + floorColorWhite && time.milliseconds() < 500) {
                            idle();
                        }
                    }
                    robot.setAllDriveMotors(0);
                }

                robot.driveDistance(0.25, 0.6);
                if(robot.colorSensor.red() >= helper.redBeaconColor){ //left side red
                    robot.pusher.setPosition(robot.pusherLeft);
                } else if(robot.colorSensor.blue() >= helper.blueBeaconColor) { //right side is red
                    robot.pusher.setPosition(robot.pusherRight);
                }
                Thread.sleep(500);
                robot.pusher.setPosition(robot.pusherMiddle);
                inFrontOfImage = false;
                doneWithFirstBeacon = true;
            }

            idle();
        }

        robot.driveDistance(-0.5, 1);
        robot.turnDegrees(0.5, 90);
        robot.setAllDriveMotors(0.5);

        while(robot.lightSensor.getLightDetected() < darkFloorValue + 0.1 && opModeIsActive()) {
            //wait for robot to run over line
            idle();
        }
        robot.setAllDriveMotors(0);
        robot.driveDistance(0.5, 0.5); //to top it off

        robot.setAllDriveMotors(0);
        robot.turnDegrees(0.5, -90);

        inFrontOfImage = false;
        while(!doneWithSecondBeacon && opModeIsActive()){
            OpenGLMatrix position = ((VuforiaTrackableDefaultListener) toolsTarget.getListener()).getPose(); //get positions
            if(position != null && !inFrontOfImage){
                VectorF translation = position.getTranslation();
                double xTrans = (double)translation.get(1); //x and y are switched for horizontal phone
//                double yTrans = (double)translation.get(0);
                double zTrans = (double)translation.get(2);

                double degreesToTurn = Math.toDegrees(Math.atan2(zTrans, xTrans)) + 90; //horizontal phone

                if(Math.abs(zTrans) > 250) {
                    double leftSpeed = Range.clip((40 + degreesToTurn * 2)/100, -Math.abs(zTrans)/2000, Math.abs(zTrans)/2000);
                    double rightSpeed = Range.clip((40 - degreesToTurn * 2)/100, -Math.abs(zTrans)/2000, Math.abs(zTrans)/2000);
                    robot.setDriveMotors(leftSpeed, leftSpeed, rightSpeed, rightSpeed);
                } else {
                    inFrontOfImage = true;
                }
            } else if(inFrontOfImage) {
                //push the button
                if(robot.lightSensor.getLightDetected() < darkFloorValue + floorColorWhite) {
                    robot.setDriveMotors(0.5, -0.5, -0.5, 0.5);
                    time.reset();
                    while(robot.lightSensor.getLightDetected() < darkFloorValue + floorColorWhite && time.milliseconds() < 250) {
                        idle();
                    }
                    if(time.milliseconds() >= 250) {
                        robot.setDriveMotors(-0.5, 0.5, 0.5, -0.5);
                        time.reset();
                        while(robot.lightSensor.getLightDetected() < darkFloorValue + floorColorWhite && time.milliseconds() < 500) {
                            idle();
                        }
                    }
                    robot.setAllDriveMotors(0);
                }

                robot.driveDistance(0.25, 0.6);
                if(robot.colorSensor.red() >= helper.redBeaconColor){ //left side red
                    robot.pusher.setPosition(robot.pusherLeft);
                } else if(robot.colorSensor.blue() >= helper.blueBeaconColor) { //right side is red
                    robot.pusher.setPosition(robot.pusherRight);
                }
                Thread.sleep(500);
                robot.pusher.setPosition(robot.pusherMiddle);
                inFrontOfImage = false;
                doneWithSecondBeacon = true;
            }

            idle();
        }
        robot.driveDistance(-0.5, 1);

        /*
        seenImage = false;
        for(VuforiaTrackable beaconImage : redTrackablesList){ //loop throught all of the trackables
            OpenGLMatrix position = ((VuforiaTrackableDefaultListener) beaconImage.getListener()).getPose(); //get positions

            if(position != null) { //if we see the object we are looking for
                seenImage = true;
                VectorF translation = position.getTranslation();
                double xTrans = (double)translation.get(1); //x and y are switched for horizontal phone
                double yTrans = (double)translation.get(0);
                double zTrans = (double)translation.get(2);

                double degreesToTurn = Math.toDegrees(Math.atan2(zTrans, xTrans)) + 90; //horizontal phone

                telemetry.addData(beaconImage.getName() + " - Translation", translation);
                telemetry.addData(beaconImage.getName() + " - Degrees", degreesToTurn);

                if(Math.abs(zTrans) > 250) {
                    double leftSpeed = Range.clip((40 + degreesToTurn * 2)/100, -Math.abs(zTrans)/2000, Math.abs(zTrans)/2000);
                    double rightSpeed = Range.clip((40 - degreesToTurn * 2)/100, -Math.abs(zTrans)/2000, Math.abs(zTrans)/2000);
                    robot.setDriveMotors(leftSpeed, leftSpeed, rightSpeed, rightSpeed);
                } else {
                    robot.setAllDriveMotors(0);
                }

            } else {
                telemetry.addData(beaconImage.getName(), "Not In View"); // if not in view it will print "Not in view"
            }
        }
        if(!seenImage) {
            //turns 45 degrees every second
            if(time.milliseconds() > 3000){
                robot.turnDegrees(0.25, 45);
                time.reset();
            }
        }

        telemetry.addData("Gyro Heading", robot.gyroSensor.getHeading());

        telemetry.update();*/
    }
}
