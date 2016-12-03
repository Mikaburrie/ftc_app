package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
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
@Autonomous(name="BIL: Blue Beacons", group="BIL")
public class BILVuforiaBlueBeacons extends LinearOpMode {

    VuforiaLocalizer vuforia;
    BILVuforiaCommon helper = new BILVuforiaCommon();
    BILRobotHardware robot = new BILRobotHardware();
    ElapsedTime time = new ElapsedTime();

    @Override public void runOpMode() throws InterruptedException{
        this.vuforia = helper.initVuforia(false, 4);
        VuforiaTrackables targets = helper.loadTargets("FTC_2016-17", "Wheels", "Tools", "Legos", "Gears");

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

        //wait for the gyro to finish calibrating if it is
        while(robot.gyroSensor.isCalibrating()){
            idle();
        }

        waitForStart();

        robot.driveDistance(0.5, 8);
        robot.turnDegrees(0.25, 45);

        targets.activate(); //activate the tracking of the image targets once the opmode starts

        List<VuforiaTrackable> blueTrackablesList = helper.returnBlueTargets(targets);

        boolean seenImage = false;
        boolean doneWithFirstBeacon = false;
        boolean doneWithSecondBeacon = false;
        boolean inFrontOfImage = false;
        VuforiaTrackable legosTarget = blueTrackablesList.get(1);
        VuforiaTrackable wheelsTarget = blueTrackablesList.get(0);

        while(!doneWithFirstBeacon && opModeIsActive()){
            OpenGLMatrix position = ((VuforiaTrackableDefaultListener) wheelsTarget.getListener()).getPose(); //get positions
            if(position != null && !inFrontOfImage){
                VectorF translation = position.getTranslation();
                double xTrans = (double)translation.get(1); //x and y are switched for horizontal phone
                double yTrans = (double)translation.get(0);
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
                robot.driveDistance(0.25, 0.5);
                if(robot.colorSensor.blue() >= 3){ //left side red
                    robot.pusher.setPosition(robot.pusherLeft);
                } else if(robot.colorSensor.red() >= 3) { //right side is red
                    robot.pusher.setPosition(robot.pusherRight);
                }
                wait(500);
                robot.pusher.setPosition(robot.pusherMiddle);
                inFrontOfImage = false;
                doneWithFirstBeacon = true;
            }

            idle();
        }

        robot.driveDistance(0.5, -1);
        robot.turnDegrees(0.5, -90);
        robot.setAllDriveMotors(0.5);

        while(robot.lightSensor.getLightDetected() < darkFloorValue + 0.1 && opModeIsActive()) {
            //wait for robot to run over line
        }

        robot.setAllDriveMotors(0);
        robot.turnDegrees(0.5, 90);

        inFrontOfImage = false;
        while(!doneWithSecondBeacon && opModeIsActive()){
            OpenGLMatrix position = ((VuforiaTrackableDefaultListener) legosTarget.getListener()).getPose(); //get positions
            if(position != null && !inFrontOfImage){
                VectorF translation = position.getTranslation();
                double xTrans = (double)translation.get(1); //x and y are switched for horizontal phone
                double yTrans = (double)translation.get(0);
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
                robot.driveDistance(0.25, 0.5);
                if(robot.colorSensor.blue() >= 3){ //left side red
                    robot.pusher.setPosition(robot.pusherLeft);
                } else if(robot.colorSensor.red() >= 3) { //right side is red
                    robot.pusher.setPosition(robot.pusherRight);
                }
                wait(500);
                robot.pusher.setPosition(robot.pusherMiddle);
                inFrontOfImage = false;
                doneWithSecondBeacon = false;
            }

            idle();
        }

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
