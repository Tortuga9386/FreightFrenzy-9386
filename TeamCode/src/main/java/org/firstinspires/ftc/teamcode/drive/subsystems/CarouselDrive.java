package org.firstinspires.ftc.teamcode.drive.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.opmodes.RobotBase;


public class CarouselDrive {
    protected HardwareMap hardwareMap;
    public Telemetry telemetry;
    public RobotBase robotBase;
    public CRServo carouselServo;

    public CarouselDrive(HardwareMap hardwareMap, RobotBase opMode) {
        this.hardwareMap = hardwareMap;
        this.telemetry = opMode.telemetry;
        this.robotBase = opMode;
        initHardware();
    }

    protected void initHardware() {
        carouselServo = hardwareMap.get(CRServo.class, "carouselServo");
        carouselServo.setDirection(DcMotorSimple.Direction.FORWARD);
        carouselServo.setPower(0.0);
    }

    public void run() {
        carouselServo.setDirection(DcMotorSimple.Direction.REVERSE);
        carouselServo.setPower(1.0);

    }

    public void runBlue() {
        carouselServo.setDirection(DcMotorSimple.Direction.FORWARD);
        carouselServo.setPower(1.0);

    }

    public boolean isBusy() {
        return carouselServo.getPower() > 0.0;
    }

    public void stop() {
        carouselServo.setPower(0.0);
    }

    public void reverse() {
        carouselServo.setDirection(DcMotorSimple.Direction.REVERSE);
        carouselServo.setPower(1.0);
    }

    public void reset() {
        initHardware();
    }

}