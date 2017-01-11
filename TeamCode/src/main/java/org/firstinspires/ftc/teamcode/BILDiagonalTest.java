package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created on 1/10/2017 by Mika.
 */
@Autonomous(name="BIL: Diagonal Test", group="BIL")
public class BILDiagonalTest extends BILAutonomousCommon {

    @Override
    public void runOpMode() throws InterruptedException{
        robot.init(hardwareMap);

        waitForStart();

        setDriveMotors(1, 0.1, 0.1, 1);

        time.reset();
        while (time.milliseconds() < 2000) {
            idle();
        }

        setAllDriveMotors(0);

    }
}
