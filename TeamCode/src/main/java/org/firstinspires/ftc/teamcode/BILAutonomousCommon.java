package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;

/**
 * Created on 1/7/2017 by Mika.
 */
public abstract class BILAutonomousCommon extends LinearOpMode {
    BILRobotHardware robot = new BILRobotHardware();
    ElapsedTime time = new ElapsedTime();

    public final static int ticksPerRotation = 1440;
    public final static double wheelCircumference = (4 * Math.PI)/12; //circumference in feet
    public final static int driveTimeScalar = 3;
    public final double lineColorThreshold = 0.04;
    double darkFloorValue = 0;
    double sideSpeed = 0.5;

    /**
     * @param mode The run mode to set for all motors.
     */
    public void setAllMotorModes(DcMotor.RunMode mode) {
        robot.motorFrontRight.setMode(mode);
        robot.motorBackRight.setMode(mode);
        robot.motorFrontLeft.setMode(mode);
        robot.motorBackLeft.setMode(mode);
    }

    /**
     *
     * @param frontLeft Power for front left wheel.
     * @param backLeft Power for back left wheel.
     * @param frontRight Power for front right wheel.
     * @param backRight Power for back right wheel.
     */
    public void setDriveMotors(double frontLeft, double backLeft, double frontRight, double backRight) {
        robot.motorFrontLeft.setPower(frontLeft);
        robot.motorBackLeft.setPower(backLeft);
        robot.motorFrontRight.setPower(frontRight);
        robot.motorBackRight.setPower(backRight);
    }

    /**
     * @param power The power to set for all drive motors.
     */
    public void setAllDriveMotors(double power) {
        robot.motorFrontLeft.setPower(power);
        robot.motorBackLeft.setPower(power);
        robot.motorFrontRight.setPower(power);
        robot.motorBackRight.setPower(power);
    }

    /**
     * @param power The speed to drive at.
     * @param distance How far the robot should travel (in feet).
     */
    public void driveDistance(double power, double distance) throws InterruptedException {
        //reset encoders just to be safe
        setAllMotorModes(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //convert input distance in feet to motor ticks
        int ticks = (int)Math.round(Math.abs(distance/wheelCircumference) * ticksPerRotation);

        //set the target positions for all motors
        robot.motorFrontLeft.setTargetPosition(ticks);
        robot.motorBackLeft.setTargetPosition(ticks);
        robot.motorFrontRight.setTargetPosition(ticks);
        robot.motorBackRight.setTargetPosition(ticks);

        //tells motors to run until position is reached
        setAllMotorModes(DcMotor.RunMode.RUN_TO_POSITION);

        //starts the motors
        setAllDriveMotors(power);

        //waits for motors to finish moving
        time.reset();
        while(getAllMotorsBusy()) {
            //if robot has been driving longer then we think necessary we will automatically stop and move on
            if(time.milliseconds() > Math.abs(ticks/power/driveTimeScalar)) {
                break;
            }
            idle();
        }

        //set all motors to 0
        setAllDriveMotors(0);

        //resets encoder values and changes mode back to default
        setAllMotorModes(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setAllMotorModes(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void driveByTime(double power, int milliseconds) throws InterruptedException {
        time.reset();
        setAllDriveMotors(power);
        while(time.milliseconds() < milliseconds) {
                idle();
        }
    }

    /**
     * @param power The power for the motors.
     * @param degrees The degrees to turn.
     */
    public void turnDegrees(double power, double degrees) throws InterruptedException {
        //set to run using encoders just in case
        setAllMotorModes(DcMotor.RunMode.RUN_USING_ENCODER);

        //gets the current heading to refer to
        int startHeading = robot.gyroSensor.getHeading();
        if((startHeading + degrees) >= 360){
            startHeading -= 360;
        }else if(startHeading + degrees < 0) {
            startHeading += 360;
        }

        //if it is more efficient to turn left
        if(degrees > 180 || degrees < 0) {
            setDriveMotors(-power, -power, power, power);
        } else {
            setDriveMotors(power, power, -power, -power);
        }

        //if we still need to turn
        while(Math.abs(Math.abs(startHeading - robot.gyroSensor.getHeading()) - Math.abs(degrees)) > 5) {
            idle();
        }

        //stop all the motors
        setAllDriveMotors(0);
    }

    public void driveUntilLineOrDistance(double power, double distance) throws InterruptedException {
        robot.lightSensor.enableLed(true);
        setAllMotorModes(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        int ticks = (int)Math.round((distance/wheelCircumference) * ticksPerRotation);

        robot.motorFrontLeft.setTargetPosition(ticks);
        robot.motorBackLeft.setTargetPosition(ticks);
        robot.motorFrontRight.setTargetPosition(ticks);
        robot.motorBackRight.setTargetPosition(ticks);

        setAllMotorModes(DcMotor.RunMode.RUN_TO_POSITION);

        setAllDriveMotors(power);

        //waits for motors to finish moving
        time.reset();
        while(getAllMotorsBusy() && robot.lightSensor.getLightDetected() < darkFloorValue + lineColorThreshold) {
            //if robot has been driving longer then we think necessary we will automatically stop and move on
            if(time.milliseconds() > ticks/power/driveTimeScalar) {
                break;
            }
            idle();
        }

        //set all motors to 0
        setAllDriveMotors(0);

        //resets encoder values and changes mode back to default
        setAllMotorModes(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setAllMotorModes(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /**
     * Strafes until a line is detected or enough time has passed. Positive power goes right, negative goes left.
     * @param power Positive goes right, negative goes left.
     * @param milliseconds The amount of time to limit to.
     * @throws InterruptedException
     */
    public void strafeUntilLineOrTime(double power, int milliseconds) throws InterruptedException {
        robot.lightSensor.enableLed(true);
        setAllMotorModes(DcMotor.RunMode.RUN_USING_ENCODER);

        //makes robot strafe right
        setDriveMotors(power, -power, -power, power);

        time.reset();

        while(time.milliseconds() < 2000){
            idle();
        }

        while(robot.lightSensor.getLightDetected() < darkFloorValue + lineColorThreshold && time.milliseconds() < milliseconds) {
            idle();
        }

        setAllDriveMotors(0);
    }


    public void diagonalUntilLineOrTime(int milliseconds) throws InterruptedException {
        robot.lightSensor.enableLed(true);
        setAllMotorModes(DcMotor.RunMode.RUN_USING_ENCODER);

        //makes the robot go diagonal
        setDriveMotors(1, 0.1, 0.1, 1);

        time.reset();
        while(robot.lightSensor.getLightDetected() < darkFloorValue + lineColorThreshold && time.milliseconds() < milliseconds) {
            idle();
        }

        setAllDriveMotors(0);
    }

    /**
     * @return If one or more motors are busy return true, otherwise false.
     */
    public boolean getAllMotorsBusy() {
        return (robot.motorFrontLeft.isBusy() || robot.motorBackLeft.isBusy() || robot.motorFrontRight.isBusy() || robot.motorBackRight.isBusy());
    }


    public void driveToTarget(VectorF translation){
        double xTrans = (double)translation.get(1); //x and y are switched for horizontal phone
        double zTrans = (double)translation.get(2);

        double degreesToTurn = Math.toDegrees(Math.atan2(zTrans, xTrans)) + 90; //horizontal phone

        double leftSpeed = Range.clip((40 + degreesToTurn * 2) / 100, -Math.abs(zTrans) / 2000, Math.abs(zTrans) / 2000);
        double rightSpeed = Range.clip((40 - degreesToTurn * 2)/100, -Math.abs(zTrans)/2000, Math.abs(zTrans)/2000);
        setDriveMotors(leftSpeed, leftSpeed, rightSpeed, rightSpeed);
    }


    public void findWhiteLine() throws InterruptedException {
        if(robot.lightSensor.getLightDetected() < darkFloorValue + lineColorThreshold) {
            setDriveMotors(sideSpeed, -sideSpeed, -sideSpeed, sideSpeed);
            time.reset();
            while(robot.lightSensor.getLightDetected() < darkFloorValue + lineColorThreshold && time.milliseconds() < 1000) {
                idle();
            }
            if(time.milliseconds() < 1000){
                setAllDriveMotors(0);
            } else {
                setDriveMotors(-sideSpeed, sideSpeed, sideSpeed, -sideSpeed);
                time.reset();
                while(robot.lightSensor.getLightDetected() < darkFloorValue + lineColorThreshold && time.milliseconds() < 2000) {
                    idle();
                }
            }
            setAllDriveMotors(0);
        }
    }


    public void getDarkFloorValue() {

    }


    public void moveToImage(VuforiaTrackable target, BILVuforiaCommon helper) throws InterruptedException {
        boolean inFrontOfImage = false;
        while(!inFrontOfImage){
            VectorF translation = helper.getTargetTranslation(target);
            if(translation != null && Math.abs(translation.get(2)) > helper.targetImageDistance) { // 2 = z
                driveToTarget(translation);
                telemetry.addData("Translation X", translation.get(0));
                telemetry.addData("Translation Y", translation.get(1));
                telemetry.addData("Translation Z", translation.get(2));
            } else {
                if(translation != null){
                    inFrontOfImage = true;
                } else {
                    telemetry.addData(target.getName() + " Target", "not in view");
                }
            }
            telemetry.update();
            idle();
        }
    }

    public void driveBetweenBeacons(int turnDegrees) throws InterruptedException
    {
        int initialTurnDegrees, finalTurnDegrees;
        if(turnDegrees > 0)
        {
            initialTurnDegrees = turnDegrees - 5;
            finalTurnDegrees = 5;
        }
        else
        {
            initialTurnDegrees = turnDegrees + 5;
            finalTurnDegrees = -5;
        }

        //turn 90 degrees the first 85 degrees at 0.5 speed, and to not overshoot the last 5 degrees would be 0.1 speed
        turnDegrees(0.5, initialTurnDegrees);
        turnDegrees(0.1, finalTurnDegrees);

        setAllDriveMotors(0.5);
        driveUntilLineOrDistance(0.5, 6);
        driveDistance(0.5, 0.5); //to top it off

        setAllDriveMotors(0);
        //turn -90 degrees the first 85 degrees at 0.5 speed, and to not overshoot the last 5 degrees would be 0.1 speed
        turnDegrees(0.5, -initialTurnDegrees);
        turnDegrees(0.1, -finalTurnDegrees);
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

        long  remaining = periodMs - (long)time.milliseconds();

        // sleep for the remaining portion of the regular cycle period.
        if (remaining > 0)
            Thread.sleep(remaining);

        // Reset the cycle clock for the next pass.
        time.reset();
    }
}