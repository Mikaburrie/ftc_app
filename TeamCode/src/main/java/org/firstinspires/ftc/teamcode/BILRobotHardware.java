package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.LightSensor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created on 10/20/2016 by Mika.
 */
public class BILRobotHardware {
    /* Public OpMode members. */
    public DcMotor motorFrontRight;
    public DcMotor motorBackRight;
    public DcMotor motorFrontLeft;
    public DcMotor motorBackLeft;
    public LightSensor lightSensor;
    public GyroSensor gyroSensor;
    public ColorSensor colorSensor;
    public Servo pusher;
    public double pusherLeft = 0.66;
    public double pusherMiddle = 0.41;
    public double pusherRight = 0.16;

    /* local OpMode members. */
    HardwareMap hwMap = null;

    /* Constructor */
    public BILRobotHardware(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        //Define and Initialize Motors
        //Left_Front, Left_Back, Right_Front, Right_Back
        motorFrontRight = hwMap.dcMotor.get("Right_Front");
        motorBackRight = hwMap.dcMotor.get("Right_Back");
        motorFrontLeft = hwMap.dcMotor.get("Left_Front");
        motorBackLeft = hwMap.dcMotor.get("Left_Back");
        motorFrontLeft.setDirection(DcMotor.Direction.REVERSE);
        motorBackLeft.setDirection(DcMotor.Direction.REVERSE);

        //Initialize Servos
        pusher = hwMap.servo.get("pusher");

        //Initialize sensors
        lightSensor = hwMap.lightSensor.get("lightSensor");
        colorSensor = hwMap.colorSensor.get("colorSensor");

        //Initialize gyro and calibrate
        gyroSensor = hwMap.gyroSensor.get("gyro");
        gyroSensor.calibrate();

        // Set all motors to zero power
        motorFrontLeft.setPower(0);
        motorBackLeft.setPower(0);
        motorFrontRight.setPower(0);
        motorBackRight.setPower(0);

        // Set all motors to run with encoders.
        motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}
