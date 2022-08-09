package org.firstinspires.ftc.teamcode.drive.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.opmodes.RobotBase;
import org.firstinspires.ftc.teamcode.drive.threads.OdometryGlobalCoordinatePosition;

import static org.firstinspires.ftc.teamcode.drive.opmodes.RobotBase.COUNTS_PER_INCH;

public class LocalizationSA {
    protected HardwareMap hardwareMap;
    public Telemetry telemetry;
    public RobotBase opMode;
    public Drive drive;
    public DcMotor leftVpodEncoder;
    public DcMotor  rightVpodEncoder;
    public DcMotor  rearHpodEncoder;
    public OdometryGlobalCoordinatePosition globalPositionUpdate;
    public Thread positionThread;
    public boolean initialized;


    public LocalizationSA(HardwareMap hardwareMap, RobotBase opMode) {
        this.hardwareMap = hardwareMap;
        this.telemetry = opMode.telemetry;
        this.opMode = opMode;
        //this.drive = new Drive(hardwareMap, opMode);
        initHardware();
    }

    protected void initHardware() {
        leftVpodEncoder  = opMode.drive.leftRearWheel;
        rightVpodEncoder = opMode.drive.leftRearWheel;
        rearHpodEncoder  = opMode.drive.rightRearWheel;

        OdometryGlobalCoordinatePosition globalPositionUpdate = new OdometryGlobalCoordinatePosition(leftVpodEncoder, rightVpodEncoder, rearHpodEncoder, COUNTS_PER_INCH, 75);
        Thread positionThread = new Thread(globalPositionUpdate);
        positionThread.start();
        initialized = true;
    }

    public void stop() {
        if (initialized && positionThread.isAlive()) {
            globalPositionUpdate.stop();
            initialized = false;
        }
    }

    public void reset() {
    }
}
