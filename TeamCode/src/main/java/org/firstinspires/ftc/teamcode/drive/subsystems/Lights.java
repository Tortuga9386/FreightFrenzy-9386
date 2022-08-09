package org.firstinspires.ftc.teamcode.drive.subsystems;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.opmodes.RobotBase;

public class Lights {
    protected HardwareMap hardwareMap;
    public Telemetry telemetry;
    private RobotBase robotBase;
    public RevBlinkinLedDriver blinkinLedDriver;

    public Lights(HardwareMap hardwareMap, RobotBase opMode) {
        this.hardwareMap = hardwareMap;
        this.robotBase = opMode;
        this.telemetry = robotBase.telemetry;
        initHardware();
    }

    protected void initHardware() {
        blinkinLedDriver = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");
    }

    public void setPattern(String patternName) {
        if (patternName != null && !patternName.trim().isEmpty()) {
            //telemetry.addData("Light Pattern:", patternName);
            blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.valueOf(patternName));
        } else {
            //telemetry.addData("Light Pattern NONE:", patternName);
        }
    }

    public void stop() {
        setPattern("BLACK");
        blinkinLedDriver.close();
    }

    public void reset() {
        setPattern("BLACK");
    }

}
