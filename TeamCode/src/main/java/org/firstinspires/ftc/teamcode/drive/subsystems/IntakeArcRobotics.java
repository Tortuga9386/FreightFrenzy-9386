package org.firstinspires.ftc.teamcode.drive.subsystems;

import com.arcrobotics.ftclib.hardware.motors.Motor.GoBILDA;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.opmodes.RobotBase;

public class IntakeArcRobotics {
    protected HardwareMap hardwareMap;
    public Telemetry telemetry;
    private RobotBase robotBase;
    public  MotorEx intakeMotor;
    public  Servo intakeFlipper;

    public IntakeArcRobotics(HardwareMap hardwareMap, RobotBase opMode) {
        this.hardwareMap = hardwareMap;
        this.robotBase = opMode;
        this.telemetry = robotBase.telemetry;
        initHardware();
    }

    protected void initHardware() {
        //Motor configuration looks a little funky here because we are using the ArcRobotics lib to maintain velocity even if battery gets low
        intakeMotor = new MotorEx(hardwareMap, "IntakeOne", GoBILDA.RPM_1620); //intakeMotor = hardwareMap.get(MotorEx.class, "IntakeOne");
        intakeMotor.setInverted(true); //.setDirection(DcMotor.Direction.FORWARD);
        intakeMotor.stopMotor();
        intakeMotor.resetEncoder(); //setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeMotor.setRunMode(MotorEx.RunMode.RawPower); //setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //intakeMotor.setZeroPowerBehavior(MotorEx.ZeroPowerBehavior.FLOAT); //setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        intakeMotor.setVeloCoefficients(1.489, 0.149, 0.0);
        intakeMotor.setFeedforwardCoefficients(0.92, 0.47);

        intakeFlipper = hardwareMap.get(Servo.class, "IntakeFlipper");
        intakeFlipper.setDirection(Servo.Direction.REVERSE);
    }

    public void run() {
        intakeMotor.set(-.9);
    }

    public boolean isBusy() {
        if (intakeMotor.get() >= 1.0) {
            return true;
        }
        return false;
    }

    public void stop() {
        intakeMotor.set(0.0);
    }

    public void reverse() {
        intakeMotor.set(1.0);
    }

    public void reset() {
        initHardware();
    }

    public void setIntakeFlipperUp() {
        intakeFlipper.setPosition(0.8);
    }

    public void setIntakeFlipperDown() {
        intakeFlipper.setPosition(0.2);
    }

}