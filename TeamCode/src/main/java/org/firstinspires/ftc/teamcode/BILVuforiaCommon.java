package org.firstinspires.ftc.teamcode;

import com.vuforia.HINT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

/**
 * Created by mikab_000 on 11/10/2016.
 */
public class BILVuforiaCommon {

    VuforiaLocalizer vuforia;

    public void initVuforia(boolean cameraPreview, int maxTrackables) {
        //Sets up camera and initializes vuforia.
        VuforiaLocalizer.Parameters parameters;
        if(cameraPreview) {
            parameters = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);
        } else {
            parameters = new VuforiaLocalizer.Parameters();
        }

        parameters.vuforiaLicenseKey =
                "AU6p3vX/////AAAAGXbGGUU5NkaliUUGq2/Kz/Z6xyNBL1fmMylvq1uMMqkFx8J4CCcF4S34r9tecpyQ0E4VJjgRklceKPzDOpiiQFxzNkZxSiIwTx4zylpK8pf6oBUg6qXji/9xlE41OyFzhKkjgCJekACepkKmviiUF94oSIRvNjOqGlVVtOKCUN3LTYF/xTIULB52NVbaXvco/9alDrM6diA/Vdfdd0qVxDnZ8mu4R7PsO19lJNcpEyNs4eSjvPlY3t75KJM1sHFFl3nUn5Piggv65VU5+sfS6VELGrGDNisMLFsp3Qk8+KSV5twxoM5cnpjGqN1CYusvkhyJAF/TBXNKb0LN0b2PNkVPIU1VYeycNax0z6HHZZ6J";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK; //uses the back camera(higher resolution)
        parameters.cameraMonitorFeedback = VuforiaLocalizer.Parameters.CameraMonitorFeedback.AXES;
        parameters.useExtendedTracking = false;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters); //creates new vuforia class with parameters
        Vuforia.setHint(HINT.HINT_MAX_SIMULTANEOUS_IMAGE_TARGETS, maxTrackables); //the max amound of image targets we are looking for
    }

    public void loadTargets(String file, String... names) {
        VuforiaTrackables imageTargets = vuforia.loadTrackablesFromAsset(file); //gets the targets from assets
        for(String name : names) {

        }
    }
}
