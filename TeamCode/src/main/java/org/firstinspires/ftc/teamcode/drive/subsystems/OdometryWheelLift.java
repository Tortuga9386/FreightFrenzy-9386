package org.firstinspires.ftc.teamcode.drive.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.opmodes.RobotBase;

public class OdometryWheelLift {
    protected HardwareMap hardwareMap;
    public Telemetry telemetry;
    public RobotBase robotBase;
    public Servo leftServo;
    public Servo rightServo;
    public Servo backServo;

    public OdometryWheelLift(HardwareMap hardwareMap, RobotBase opMode) {
        this.hardwareMap = hardwareMap;
        this.telemetry = opMode.telemetry;
        this.robotBase = opMode;
        initHardware();
    }

    protected void initHardware() {
        leftServo = hardwareMap.get(Servo.class, "leftOdoServo");
        rightServo = hardwareMap.get(Servo.class, "rightOdoServo");
        backServo = hardwareMap.get(Servo.class, "backOdoServo");
    }

    public void down() {
        leftServo.setPosition(-0.50);
        rightServo.setPosition(1.5);
        backServo.setPosition(1.5);
    }

    public void up(){
        leftServo.setPosition(1.5);
        rightServo.setPosition(-0.5);
        backServo.setPosition(-0.5);
    }

    public void stop() {

    }

    public void reverse() {

    }

}