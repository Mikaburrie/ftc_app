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
    public DcMotor motorLift;
    public LightSensor lightSensor;
    public GyroSensor gyroSensor;
    public ColorSensor colorSensor;
    public Servo pusher;
    public double pusherLeft = 0.80;
    public double pusherMiddle = 0.45;
    public double pusherRight = 0.10;
    public Servo liftHolder;
    public double liftHolderStart = 0.2;
    public double liftHolderRelease = 1;

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

        //Lift Motor
        motorLift = hwMap.dcMotor.get("Lift");

        //Initialize Servos
        pusher = hwMap.servo.get("pusher");
        pusher.setPosition(pusherMiddle);
        liftHolder = hwMap.servo.get("liftHolder");
        liftHolder.setPosition(liftHolderStart);

        //Initialize sensors
        lightSensor = hwMap.lightSensor.get("lightSensor");
        colorSensor = hwMap.colorSensor.get("colorSensor");

        //Initialize gyro and calibrate
        gyroSensor = hwMap.gyroSensor.get("gyro");

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
