package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.Map;
import java.util.Set;

/**
 * Created by nill on 1/7/16.
 */
public class BILSimpleForward extends OpMode
{
    DcMotor motorRight;
    DcMotor motorLeft;
    DcMotorController wheelController;

    Servo claw;
    @Override
    public void init()
    {
        motorRight = hardwareMap.dcMotor.get("motor_2");
        motorLeft = hardwareMap.dcMotor.get("motor_1");
        wheelController = hardwareMap.dcMotorController.get("motor control");
        motorLeft.setDirection(DcMotor.Direction.REVERSE);
    }

    @Override
    public void loop() {
        motorRight.setPower(1.0);
        motorLeft.setPower(1.0);
        telemetry.addData("Text", "free flow text");
        telemetry.addData("left motor", motorLeft.getPower());
        telemetry.addData("right motor", motorRight.getPower());
        telemetry.addData("RunMode: ", motorLeft.getMode().toString());
    }
}
