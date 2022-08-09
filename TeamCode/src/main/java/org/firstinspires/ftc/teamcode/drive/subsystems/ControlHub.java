package org.firstinspires.ftc.teamcode.drive.subsystems;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.opmodes.RobotBase;

public class ControlHub {
    protected HardwareMap hardwareMap;
    public Telemetry telemetry;
    private RobotBase opMode;
    public BNO055IMU imu;
    protected BNO055IMU.Parameters parameters;
    public boolean initialized = false;

    public ControlHub(HardwareMap hardwareMap, RobotBase opMode) {
        this.hardwareMap = hardwareMap;
        this.telemetry = opMode.telemetry;
        this.opMode = opMode;
        initHardware();
    }

    protected void initHardware() {
        // Retrieve and initialize the IMU. We expect the IMU to be attached to I2C Bus 0
        // from the Control Hub (internally) and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        parameters = new BNO055IMU.Parameters();
        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.RADIANS;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;
        imu.initialize(parameters);
        initialized                    = true;
    }

    public void stop() {
        if (initialized) {
            imu.close();
            initialized = false;
        }
    }

    public void reset() {
    }
}
