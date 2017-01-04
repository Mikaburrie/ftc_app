package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.List;

/**
 * Created by mikab_000 on 12/31/2016.
 */
@Autonomous(name="BIL: Gears Tracking Test", group="BIL")
public class BILVuforiaGearsPicTest extends LinearOpMode {

    VuforiaLocalizer vuforia;
    BILVuforiaCommon helper = new BILVuforiaCommon();
    BILRobotHardware robot = new BILRobotHardware();

    @Override public void runOpMode() throws InterruptedException{
        this.vuforia = helper.initVuforia(false, 4);
        VuforiaTrackables targets = helper.loadTargets("FTC_2016-17", "Wheels", "Tools", "Legos", "Gears");

        robot.init(hardwareMap);

        robot.lightSensor.enableLed(true);

        VuforiaTrackable gearsTarget = targets.get(3);

        waitForStart();

        targets.activate();

        boolean imageSeen = false;
        while(!imageSeen){
            OpenGLMatrix position = ((VuforiaTrackableDefaultListener) gearsTarget.getListener()).getPose(); //get positions
            if(position != null){
                VectorF translation = position.getTranslation();
                telemetry.addData("TranslationX", translation.get(0));
                telemetry.addData("TranslationY", translation.get(1));
                telemetry.addData("TranslationZ", translation.get(2));
                if(translation != null && Math.abs(translation.get(2)) > helper.targetImageDistance) {
                    helper.driveToTarget(gearsTarget, robot);
                } else {
                    if(translation != null){
                        telemetry.addData("Finished", "Done");
                    } else {
                        telemetry.addData("Gears Target", "not in view");
                    }
                }
            }

            telemetry.update();
            idle();
        }
    }
}
