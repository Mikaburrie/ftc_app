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
public class BILVuforiaImageRecognition extends LinearOpMode {

    VuforiaLocalizer vuforia;
    BILVuforiaCommon helper = new BILVuforiaCommon();
    BILRobotHardware robot = new BILRobotHardware();

    @Override public void runOpMode() throws InterruptedException {
        //Sets up camera and initializes vuforia.
//        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);
//        parameters.vuforiaLicenseKey =
//                "AU6p3vX/////AAAAGXbGGUU5NkaliUUGq2/Kz/Z6xyNBL1fmMylvq1uMMqkFx8J4CCcF4S34r9tecpyQ0E4VJjgRklceKPzDOpiiQFxzNkZxSiIwTx4zylpK8pf6oBUg6qXji/9xlE41OyFzhKkjgCJekACepkKmviiUF94oSIRvNjOqGlVVtOKCUN3LTYF/xTIULB52NVbaXvco/9alDrM6diA/Vdfdd0qVxDnZ8mu4R7PsO19lJNcpEyNs4eSjvPlY3t75KJM1sHFFl3nUn5Piggv65VU5+sfS6VELGrGDNisMLFsp3Qk8+KSV5twxoM5cnpjGqN1CYusvkhyJAF/TBXNKb0LN0b2PNkVPIU1VYeycNax0z6HHZZ6J";
//        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK; //uses the back camera(higher resolution)
//        parameters.cameraMonitorFeedback = VuforiaLocalizer.Parameters.CameraMonitorFeedback.AXES;
//        parameters.useExtendedTracking = false;
//        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters); //creates new vuforia class with parameters
//        Vuforia.setHint(HINT.HINT_MAX_SIMULTANEOUS_IMAGE_TARGETS, 4); //the max amound of image targets we are looking for
        this.vuforia = helper.initVuforia(false, 4);


//        VuforiaTrackables imageTargets = vuforia.loadTrackablesFromAsset("FTC_2016-17"); //gets the targets from assets
//        imageTargets.get(0).setName("Wheels"); //name first one wheels
//        imageTargets.get(1).setName("Tools"); //second is tools
//        imageTargets.get(2).setName("Legos"); //third is legos
//        imageTargets.get(3).setName("Gears"); //fourth is gears
        VuforiaTrackables imageTargets = helper.loadTargets("FTC_2016-17", "Wheels", "Tools", "Legos", "Gears");

    //     imageTargets.get(0).setLocation(createMatrix(0, 0, 0, 0, 0, 0));

        robot.init(hardwareMap);

        waitForStart(); //waits for the op mode to be started

        imageTargets.activate(); //activate the tracking of the image targets once the opmode starts

        boolean seenImage = false;
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
                        robot.setDriveMotors(leftSpeed, leftSpeed, rightSpeed, rightSpeed);
                    } else {
                        robot.setAllDriveMotors(0);
                    }

                } else {
                    telemetry.addData(beaconImage.getName(), "Not In View"); // if not in view it will print "Not in view"
                }
            }
            if(!seenImage)
            {
                robot.setAllDriveMotors(0);
            }
            telemetry.update();
        }





        /** For convenience, gather together all the trackable objects in one easily-iterable collection */
/*        List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();
        allTrackables.addAll(imageTargets);
k

        float mmPerInch        = 25.4f;
        float mmBotWidth       = 18 * mmPerInch;            // ... or whatever is right for your robot
        float mmFTCFieldWidth  = (12*12 - 2) * mmPerInch;   // the FTC field is ~11'10" center-to-center of the glass panels

        */
    }

    public OpenGLMatrix createMatrix(float x, float y, float z, float u, float v, float w){


        return OpenGLMatrix.translation(x, y, x).
                multiplied(Orientation.getRotationMatrix(AxesReference.EXTRINSIC, AxesOrder.XYX, AngleUnit.DEGREES, u, v, w));
    }
}
