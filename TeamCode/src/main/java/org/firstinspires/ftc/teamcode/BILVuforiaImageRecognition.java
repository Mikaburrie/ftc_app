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

    @Override public void runOpMode() throws InterruptedException {
        //Sets up camera and initializes vuforia.
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);
        parameters.vuforiaLicenseKey =
                "AU6p3vX/////AAAAGXbGGUU5NkaliUUGq2/Kz/Z6xyNBL1fmMylvq1uMMqkFx8J4CCcF4S34r9tecpyQ0E4VJjgRklceKPzDOpiiQFxzNkZxSiIwTx4zylpK8pf6oBUg6qXji/9xlE41OyFzhKkjgCJekACepkKmviiUF94oSIRvNjOqGlVVtOKCUN3LTYF/xTIULB52NVbaXvco/9alDrM6diA/Vdfdd0qVxDnZ8mu4R7PsO19lJNcpEyNs4eSjvPlY3t75KJM1sHFFl3nUn5Piggv65VU5+sfS6VELGrGDNisMLFsp3Qk8+KSV5twxoM5cnpjGqN1CYusvkhyJAF/TBXNKb0LN0b2PNkVPIU1VYeycNax0z6HHZZ6J";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK; //uses the back camera(higher resolution)
        parameters.cameraMonitorFeedback = VuforiaLocalizer.Parameters.CameraMonitorFeedback.AXES;
        parameters.useExtendedTracking = false;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters); //creates new vuforia class with parameters
        Vuforia.setHint(HINT.HINT_MAX_SIMULTANEOUS_IMAGE_TARGETS, 4); //the max amound of image targets we are looking for


        VuforiaTrackables imageTargets = vuforia.loadTrackablesFromAsset("FTC_2016-17"); //gets the targets from assets
        imageTargets.get(0).setName("Wheels"); //name first one wheels
        imageTargets.get(1).setName("Tools"); //second is tools
        imageTargets.get(2).setName("Legos"); //third is legos
        imageTargets.get(3).setName("Gears"); //fourth is gears

    //     imageTargets.get(0).setLocation(createMatrix(0, 0, 0, 0, 0, 0));

        waitForStart(); //waits for the op mode to be started

        imageTargets.activate(); //activate the tracking of the image targets once the opmode starts

        while(opModeIsActive()){ //when the op mode is active
            for(VuforiaTrackable beaconImage : imageTargets){ //loop throught all of the trackables
                OpenGLMatrix position = ((VuforiaTrackableDefaultListener) beaconImage.getListener()).getPose(); //get positions

                if(position != null){ //if we see the object we are looking for
                    VectorF translation = position.getTranslation();

                    telemetry.addData(beaconImage.getName() + " - Translation", translation);

                    double degreesToTurn = Math.toDegrees(Math.atan2(translation.get(1), translation.get(0))) - 90; //vertical phone

                    telemetry.addData(beaconImage.getName() + " - Degrees", degreesToTurn);
                } else {
                    telemetry.addData(beaconImage.getName(), "Not In View"); // if not in view it will print "Not in view"
                }
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
