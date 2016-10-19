package org.firstinspires.ftc.teamcode;

import android.os.DropBoxManager;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import java.util.Map.Entry;
import java.util.Set;

/**
 * Created by nill on 12/29/15.
 */
public class BILSimpleReader extends OpMode
{
   //  DcMotorController.DeviceMode devMode;
    DcMotorController wheelController;
    DcMotor motorRight;
    DcMotor motorLeft;

    @Override public void init ()
    {
        /* IGN Commented out so that 2016-2017 code base works.
        motorRight = hardwareMap.dcMotor.get("motor_2");
        motorLeft = hardwareMap.dcMotor.get("motor_1");
        motorLeft.setDirection(DcMotor.Direction.REVERSE);
       wheelController = hardwareMap.dcMotorController.get("motor control");
       wheelController.setMotorControllerDeviceMode(DcMotorController.DeviceMode.READ_ONLY);
        Set<Entry<String, DcMotorController>> controllerSet = hardwareMap.dcMotorController.entrySet();
        for (Entry<String, DcMotorController> entry: controllerSet)
        {
            System.out.println("Entry key:" + entry.getKey());
            System.out.println("Entry value:" + entry.getValue());
        }
        */
    }

    @Override public void loop ()
    {
        telemetry.addData("Text", "free flow text");
        telemetry.addData("left motor", motorLeft.getPower());
        telemetry.addData("right motor", motorRight.getPower());
        // telemetry.addData("wheel controller", wheelController.getMotorControllerDeviceMode());
        telemetry.addData("RunMode: ", motorLeft.getMode().toString());
        telemetry.addData("motor left current position", motorLeft.getCurrentPosition());
        telemetry.addData("motor left current position", motorRight.getCurrentPosition());
    }
}
