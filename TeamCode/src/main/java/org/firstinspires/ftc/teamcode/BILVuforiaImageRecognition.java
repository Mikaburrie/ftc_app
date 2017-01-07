package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.vuforia.HINT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.ArrayList;
import java.util.List;

/**
 * Created by mikab_000 on 11/3/2016.
 */

@Autonomous(name="BIL: Vuforia Image Recognition", group="BIL")
public class BILVuforiaImageRecognition extends BILAutonomousCommon {

    VuforiaLocalizer vuforia;
    BILVuforiaCommon helper = new BILVuforiaCommon();

    @Override public void runOpMode() throws InterruptedException {
        this.vuforia = helper.initVuforia(false, 4);
        VuforiaTrackables imageTargets = helper.loadTargets("FTC_2016-17", "Wheels", "Tools", "Legos", "Gears");

        robot.init(hardwareMap);

        waitForStart(); //waits for the op mode to be started

        imageTargets.activate(); //activate the tracking of the image targets once the opmode starts

        boolean seenImage;
        while(opModeIsActive()) { //when the op mode is active
            seenImage = false;
            for(VuforiaTrackable beaconImage : imageTargets){ //loop throught all of the trackables
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
                        double leftSpeed = (40 + degreesToTurn * 2)/100;
                        double rightSpeed = (40 - degreesToTurn * 2)/100;
                        setDriveMotors(leftSpeed, leftSpeed, rightSpeed, rightSpeed);
                    } else {
                        setAllDriveMotors(0);
                    }

                } else {
                    telemetry.addData(beaconImage.getName(), "Not In View"); // if not in view it will print "Not in view"
                }
            }
            if(!seenImage)
            {
                setAllDriveMotors(0);
            }
            telemetry.update();
        }
    }
}
