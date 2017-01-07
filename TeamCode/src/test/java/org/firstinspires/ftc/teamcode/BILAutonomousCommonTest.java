package org.firstinspires.ftc.teamcode;

import org.junit.After;
import org.junit.Assert;
import org.junit.Before;
import org.junit.Test;

import static org.junit.Assert.*;

/**
 * Created by INill on 12/15/2015.
 */
public class BILAutonomousCommonTest {

    private BILAutonomousCommon2015 bilAutonomousCommon;

    @Before
    public void setUp() {
        bilAutonomousCommon = new BILEncoderAutonomousLeft(); // Anything that inherits from common is fine here.
    }

    @After
    public void tearDown() {
        bilAutonomousCommon = null;
    }

    @Test
    public void testSetForwardDriveDistance() {

        // Set up
        double distance = 2.0; // feet?
        double circumference = 3.5; // inches
        double expectedValue = 9874.0;

        // Run the test
        bilAutonomousCommon.setForwardDriveDistance(distance, circumference); // feet and inches?

        // Verify
        Assert.assertEquals("Encoder ticks value not same as expected", expectedValue, bilAutonomousCommon.encoderTicks, 0.01);
    }
}