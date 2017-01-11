package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by mikab_000 on 1/10/2017.
 */
@Autonomous(name="BIL: Strafe Test", group="BIL")
public class BILStrafeTest extends BILAutonomousCommon{

    @Override
    public void runOpMode() throws InterruptedException{
        robot.init(hardwareMap);

        waitForStart();

        setDriveMotors(0.5, -0.5, -0.5, 0.5);

        time.reset();

        while(time.milliseconds() < 2000){
            idle();
        }
        setAllDriveMotors(0);

    }
}

