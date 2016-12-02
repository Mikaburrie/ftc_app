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
        robot.turnDegrees(0.25, -135);

        targets.activate(); //activate the tracking of the image targets once the opmode starts

        List<VuforiaTrackable> redTrackablesList = helper.returnRedTargets(targets);

        boolean seenImage = false;
        boolean doneWithFirstBeacon = false;
        boolean inFrontOfImage = false;
        VuforiaTrackable toolsTarget = redTrackablesList.get(1);

        while(opModeIsActive()) { //when the op mode is active

            while(!doneWithFirstBeacon){
                OpenGLMatrix position = ((VuforiaTrackableDefaultListener) toolsTarget.getListener()).getPose(); //get positions
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
                }

                idle();
            }

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

            telemetry.update();
        }
    }
}
