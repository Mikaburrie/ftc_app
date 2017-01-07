package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.Map;
import java.util.Set;

/**
 * Created by nill on 11/19/15.
 */
public class BILInfinite extends BILAutonomousCommon2015 {
    //DcMotor motorRight;
    //DcMotor motorLeft;

    Servo claw;

    private int v_state;

    @Override
    public void init() {
        super.init();
        resetDriveEncoders();
        v_state = 0;
       /* Set<Map.Entry<String, DcMotorController>> controllerSet = hardwareMap.dcMotorController.entrySet();
        for (Map.Entry<String, DcMotorController> entry : controllerSet) {
            System.out.println("Entry key:" + entry.getKey());
            System.out.println("Entry value:" + entry.getValue());
        }*/
    }
    /*
	 * This method will be called repeatedly in a loop
	 *
	 * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#run()
	 */

    @Override
    public void loop() {
        switch (v_state) {
            case 0:
                if (!gyroSensor.isCalibrating()){
                    v_state++;
                }
                break;
            case 1:
                resetDriveEncoders();
                runUsingEncoders();
                setForwardDriveDistance(70.0, circum); //drives backwards for 45 feet
                calculateSpeedBasedOnGyro();
                driveForward(speedContainer);

                if (haveDriveEncodersReached() || getDistance() < 10)
                {
                    stopDriving();

                    v_state++;
                }
                break;
           /* case 1:
                System.out.println("********************** " + v_state);
                resetDriveEncoders();
                if (areDriveEncodersZero()) {
                    v_state++;
                }

                break;
            case 2:
                System.out.println("*********** " + v_state);
                runUsingEncoders();
                setForwardDriveDistance(2.5, circum);
                driveForward(); //drive forward 2 feet

                if (haveDriveEncodersReached()) {
                    stopDriving();
                    v_state++;
                }
                break;
            case 3:
                resetDriveEncoders();
                System.out.println("********************** " + v_state);
                resetDriveEncoders();
                if (areDriveEncodersZero()) {
                    v_state++;
                }

                break;
            case 4:
                runUsingEncoders();
                setForwardDriveDistance(3.0, circum);
                turnLeft(); //turn left without moving forward

                if (hasLeftDriveEncoderReached(encoderTicks)) {
                    stopDriving();
                    v_state++;
                }
                break;
            case 5:
                resetDriveEncoders();
                System.out.println("********************** " + v_state);
                resetDriveEncoders();
                if (areDriveEncodersZero()) {
                    v_state++;
                }

                break;
            case 6:
                runUsingEncoders();
                setForwardDriveDistance(5.0, circum);
                driveUpRamp(); //drive forward up ramp

                if (haveDriveEncodersReached()) {
                    stopDriving();
                    v_state++;
                }
                break; */

        }
        telemetry.addData("State", v_state);
    }

    private void printToLog() {
        System.out.println("left motor: " + motorLeft.getCurrentPosition());
        System.out.println("right motor: " + motorRight.getCurrentPosition());
        System.out.println("encoders mode: " + motorLeft.getMode());
        // System.out.println("wheel controller: " + wheelController.getMotorControllerDeviceMode());
    }
}
