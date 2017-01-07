package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

/**
 * Created on 12/31/2016 by Mika.
 */
@Autonomous(name="BIL: Gears Tracking Test", group="BIL")
public class BILVuforiaGearsPicTest extends BILAutonomousCommon {

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

            VectorF translation = helper.getTargetTranslation(gearsTarget);

            if(translation != null && Math.abs(translation.get(2)) > helper.targetImageDistance) {
                helper.driveToTarget(gearsTarget, robot);
            } else {
                if(translation != null){
                    telemetry.addData("Finished", "Done");
                    setAllDriveMotors(0);
                    imageSeen = true;
                } else {
                    telemetry.addData("Gears Target", "not in view");
                }
            }

            telemetry.update();
            idle();
        }
    }
}
