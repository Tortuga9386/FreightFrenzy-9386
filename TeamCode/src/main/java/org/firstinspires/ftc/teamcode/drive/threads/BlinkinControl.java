package org.firstinspires.ftc.teamcode.drive.threads;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;

import org.firstinspires.ftc.robotcore.internal.system.Deadline;

import java.util.concurrent.TimeUnit;

/**
 * Created by mchilek on 1/19/2020.
 * 
 * Display patterns through a REV Robotics Blinkin LED Driver.
 * AUTO mode cycles through all of the patterns.
 * MANUAL mode allows the user to manually change patterns using the
 * left and right bumpers of a gamepad.
 *
 * Configure the driver on a servo port, and name it "blinkin".
 *
 * Displays the first pattern upon init.
 */
public class BlinkinControl implements Runnable{
    
    //Servo controller
    private RevBlinkinLedDriver blinkinLedDriver;
    private RevBlinkinLedDriver.BlinkinPattern pattern;

    //Thead run condition
    private boolean isRunning = true;

    //Rate limit updates to every 500ms
    private final static int UPDATE_LOCKOUT = 500;
    Deadline ledCycleDeadline;
    Deadline updateRateLimit;

    public BlinkinControl(RevBlinkinLedDriver blinkinLedDriver, String patternName){
        
        this.blinkinLedDriver = blinkinLedDriver;
        
        blinkinLedDriver.resetDeviceConfigurationForOpMode();
        
        updateRateLimit = new Deadline(UPDATE_LOCKOUT, TimeUnit.MILLISECONDS);

        updateBlinkinDisplay(patternName);
    }

    /*
     * updateBlinkinDisplay
     * 
     */
    protected void updateBlinkinDisplay(String patternName)
    {
        if (!updateRateLimit.hasExpired()) {
            return;
        }

        if (patternName != null && !patternName.trim().isEmpty()) {
            pattern = RevBlinkinLedDriver.BlinkinPattern.valueOf(patternName);
            blinkinLedDriver.setPattern(pattern);
            updateRateLimit.reset();
        } 
    }

    //display = telemetry.addData("Display Kind: ", displayKind.toString());
    //patternName = telemetry.addData("Pattern: ", pattern.toString());

    /**
     * Stops the position update thread
     */
    public void stop(){ 
        blinkinLedDriver.close();
        isRunning = false;
    }
    
    /**
     * Runs the thread
     */
    @Override
    public void run() {
        while(isRunning) {
            updateBlinkinDisplay("");
        }
    }
}
