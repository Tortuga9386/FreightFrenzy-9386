package org.firstinspires.ftc.teamcode.drive.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.opmodes.RobotBase;

public class Shooter {
    protected HardwareMap hardwareMap;
    public Telemetry telemetry;
    public RobotBase robotBase;
    public DcMotorEx shooterMotorOne ;
    public Servo triggerServo;
    private double targetVelocity = 100;
    private boolean triggerLockout = false;

    public Shooter(HardwareMap hardwareMap, RobotBase opMode) {
        this.hardwareMap = hardwareMap;
        this.telemetry = opMode.telemetry;
        this.robotBase = opMode;
        initHardware();
    }

    protected void initHardware() {
        shooterMotorOne = hardwareMap.get(DcMotorEx.class, "MotorOne");
        shooterMotorOne.setDirection(DcMotor.Direction.REVERSE);
        shooterMotorOne.setPower(0.0);
        shooterMotorOne.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterMotorOne.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterMotorOne.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        triggerServo = hardwareMap.get(Servo.class, "ShooterServo");
    }

    public void run() {
        shooterMotorOne.setPower(1.0);
    }

    public boolean isBusy() {
        if (shooterMotorOne.getVelocity() >= 0.0) {
            return true;
        }
        return false;
    }

    public void stop() {
        shooterMotorOne.setPower(0.0);
    }

    public void reverse() {
        shooterMotorOne.setPower(-1.0);
    }

    public void reset() {
        initHardware();
    }

    private void triggerForward() {
        triggerServo.setPosition(0.45);
    }

    private void triggerBack() {
        triggerServo.setPosition(0.0);
    }

    public void triggerRun() { triggerShoot(); }

    public void triggerStop() { triggerBack(); }

    public void triggerShoot() {
        triggerForward();
        this.robotBase.util.sleep(800);
        triggerBack();
        this.robotBase.util.sleep(1200);
    }

}