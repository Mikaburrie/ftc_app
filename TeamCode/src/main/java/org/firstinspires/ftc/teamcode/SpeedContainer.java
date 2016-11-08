package org.firstinspires.ftc.teamcode;

/**
 * Created by nill on 2/18/16.
 */
public class SpeedContainer {
    private double rightSpeed;
    private double leftSpeed;

    public SpeedContainer()
    {
        this.rightSpeed = 0;
        this.leftSpeed = 0;
    }

    public double getLeftSpeed() {
        return leftSpeed;
    }

    public void setLeftSpeed(double leftSpeed) { this.leftSpeed = leftSpeed; }

    public double getRightSpeed() {
        return rightSpeed;
    }

    public void setRightSpeed(double rightSpeed) {
        this.rightSpeed = rightSpeed;
    }
}
