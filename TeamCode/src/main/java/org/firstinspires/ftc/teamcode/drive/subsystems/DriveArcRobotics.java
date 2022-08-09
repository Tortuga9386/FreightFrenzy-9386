package org.firstinspires.ftc.teamcode.drive.subsystems;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.opmodes.RobotBase;


public class DriveArcRobotics {
    protected HardwareMap hardwareMap;
    public Telemetry telemetry;
    private RobotBase robotBase;
    private MecanumDrive mecanumDrive;
    protected Util util;
    public Motor leftFrontWheel;
    public Motor rightFrontWheel;
    public Motor leftRearWheel;
    public Motor rightRearWheel;
    public boolean turtleMode = false;

    public DriveArcRobotics(HardwareMap hardwareMap, RobotBase opMode) {
        this.hardwareMap = hardwareMap;
        this.robotBase = opMode;
        this.telemetry = robotBase.telemetry;
        initHardware();
    }

    protected void initHardware() {
        leftFrontWheel = new Motor(hardwareMap, "LeftFrontWheel");
        leftFrontWheel.setRunMode(Motor.RunMode.RawPower);
        leftFrontWheel.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        rightFrontWheel = new Motor(hardwareMap, "RightFrontWheel");
        rightFrontWheel.setRunMode(Motor.RunMode.RawPower);
        rightFrontWheel.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        leftRearWheel = new Motor(hardwareMap, "LeftRearWheel");
        leftRearWheel.setRunMode(Motor.RunMode.RawPower);
        leftRearWheel.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        rightRearWheel = new Motor(hardwareMap, "RightRearWheel");
        rightRearWheel.setRunMode(Motor.RunMode.RawPower);
        rightRearWheel.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        mecanumDrive = new MecanumDrive(leftFrontWheel, rightFrontWheel, leftRearWheel, rightRearWheel);
        mecanumDrive.setRightSideInverted(true);
    }

    public void driveRobotCentric(double moveX, double moveY, double moveRotation) {
        setTurtleMode();
        mecanumDrive.driveRobotCentric(moveX, moveY, moveRotation,true);
    }

    public void driveFieldCentric(double moveX, double moveY, double moveRotation, double heading) {
        setTurtleMode();
        mecanumDrive.driveFieldCentric(moveX, moveY, moveRotation, Math.toRadians(heading));
    }

    public void setTurtleMode() {
        if (turtleMode) {
            mecanumDrive.setMaxSpeed(0.5);
        } else {
            mecanumDrive.setMaxSpeed(1.0);
        }
    }

    public void stop() {
        mecanumDrive.stop();
    }

    public void reset() {

    }
}
