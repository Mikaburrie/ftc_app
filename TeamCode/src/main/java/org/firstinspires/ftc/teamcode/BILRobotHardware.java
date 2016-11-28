package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.LightSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by mikab_000 on 10/20/2016.
 */
public class BILRobotHardware {
    /* Public OpMode members. */
    public DcMotor motorFrontRight;
    public DcMotor motorBackRight;
    public DcMotor motorFrontLeft;
    public DcMotor motorBackLeft;
    public LightSensor lightSensor;
    public Servo pusher;
    public double pusherLeft = 0.66;
    public double pusherMiddle = 0.41;
    public double pusherRight = 0.16;
    public final static int ticksPerRotation = 1440;
    public final static double wheelCircumference = (4 * Math.PI)/12; //circumference in feet

    /* local OpMode members. */
    HardwareMap hwMap          =  null;
    private ElapsedTime period = new ElapsedTime();

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

        // Set all motors to zero power
        setAllDriveMotors(0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        setAllMotorModes(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /**
     * @param mode The run mode to set for all motors.
     */
    public void setAllMotorModes(DcMotor.RunMode mode) {
        motorFrontRight.setMode(mode);
        motorBackRight.setMode(mode);
        motorFrontLeft.setMode(mode);
        motorBackLeft.setMode(mode);
    }

    /**
     *
     * @param frontLeft Power for front left wheel.
     * @param backLeft Power for back left wheel.
     * @param frontRight Power for front right wheel.
     * @param backRight Power for back right wheel.
     */
    public void setDriveMotors(double frontLeft, double backLeft, double frontRight, double backRight) {
        motorFrontLeft.setPower(frontLeft);
        motorBackLeft.setPower(backLeft);
        motorFrontRight.setPower(frontRight);
        motorBackRight.setPower(backRight);
    }

    /**
     * @param power The power to set for all drive motors.
     */
    public void setAllDriveMotors(double power) {
        motorFrontLeft.setPower(power);
        motorBackLeft.setPower(power);
        motorFrontRight.setPower(power);
        motorBackRight.setPower(power);
    }

    /**
     * @param power The speed to drive at.
     * @param distance How far the robot should travel (in feet).
     */
    public void driveDistance(double power, double distance) {
        //reset encoders just to be safe
        setAllMotorModes(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //convert input distance in feet to motor ticks
        int ticks = (int)Math.round((distance/wheelCircumference) * ticksPerRotation);

        //set the target positions for all motors
        motorFrontLeft.setTargetPosition(ticks);
        motorBackLeft.setTargetPosition(ticks);
        motorFrontRight.setTargetPosition(ticks);
        motorBackRight.setTargetPosition(ticks);

        //tells motors to run until position is reached
        setAllMotorModes(DcMotor.RunMode.RUN_TO_POSITION);

        //starts the motors
        setAllDriveMotors(power);

        //waits for motors to finish moving
        while(!getAllMotorsBusy()) {/*nothing*/}

        //set all motors to 0
        setAllDriveMotors(0);

        //resets encoder values and changes mode back to default
        setAllMotorModes(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setAllMotorModes(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /**
     * @return If one or more motors are busy return true, otherwise false.
     */
    public boolean getAllMotorsBusy() {
        return (motorFrontLeft.isBusy() || motorBackLeft.isBusy() || motorFrontRight.isBusy() || motorBackRight.isBusy());
    }

    /***
     *
     * waitForTick implements a periodic delay. However, this acts like a metronome with a regular
     * periodic tick.  This is used to compensate for varying processing times for each cycle.
     * The function looks at the elapsed cycle time, and sleeps for the remaining time interval.
     *
     * @param periodMs  Length of wait cycle in mSec.
     * @throws InterruptedException
     */
    public void waitForTick(long periodMs) throws InterruptedException {

        long  remaining = periodMs - (long)period.milliseconds();

        // sleep for the remaining portion of the regular cycle period.
        if (remaining > 0)
            Thread.sleep(remaining);

        // Reset the cycle clock for the next pass.
        period.reset();
    }
}
