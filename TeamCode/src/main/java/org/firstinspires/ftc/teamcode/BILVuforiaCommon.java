package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.Range;
import com.vuforia.HINT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
//import org.firstinspires.ftc.robotcore.internal.VuforiaTrackablesImpl;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/**
 * Created on 11/10/2016 by Mika.
 */
public class BILVuforiaCommon {

    VuforiaLocalizer vuforia;
    int redBeaconColor = 2;
    int blueBeaconColor = 4;
    int targetImageDistance = 90;

    public VuforiaLocalizer initVuforia(boolean cameraPreview, int maxTrackables) {
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
        return vuforia;
    }

    public VuforiaTrackables loadTargets(String file, String... names) {
        VuforiaTrackables imageTargets = vuforia.loadTrackablesFromAsset(file); //gets the targets from assets
        int i = 0;
        for(String name : names) {
            imageTargets.get(i).setName(name);
            i++;
        }
        return imageTargets;
    }

    public List<VuforiaTrackable> returnRedTargets(VuforiaTrackables allTargets) {

        return new ArrayList<>(Arrays.asList(allTargets.get(1), allTargets.get(3)));
    }

    public List<VuforiaTrackable> returnBlueTargets(VuforiaTrackables allTargets) {

        return new ArrayList<>(Arrays.asList(allTargets.get(0), allTargets.get(2)));
    }

    public VectorF getTargetTranslation(VuforiaTrackable target){
        OpenGLMatrix position = ((VuforiaTrackableDefaultListener) target.getListener()).getPose(); //get positions
        if(position != null){
            return position.getTranslation();
        }else{
            return null;
        }
    }
}
