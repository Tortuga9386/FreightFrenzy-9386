package org.firstinspires.ftc.teamcode.drive.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.opmodes.AutoPath;
import org.firstinspires.ftc.teamcode.drive.opmodes.Calibration;
import org.firstinspires.ftc.teamcode.drive.opmodes.MainAutonOp;
import org.firstinspires.ftc.teamcode.drive.opmodes.RobotBase;

import java.util.Arrays;
import java.util.Collections;

import static org.firstinspires.ftc.teamcode.drive.subsystems.GlobalVar.getAllianceColor;

public class Drive {
    public static boolean RAMP_DRIVE_POWER      = true;
    public static double  RAMP_DRIVE_DURATION   = 0.02;
    protected HardwareMap hardwareMap;
    public Telemetry telemetry;
    private RobotBase robotBase;
    protected Util util;
    public DcMotor leftFrontWheel;
    public DcMotor rightFrontWheel;
    public DcMotor leftRearWheel;
    public DcMotor rightRearWheel;
    public boolean turtleMode = false;
    private double prevRRWPower = 0;
    private double prevLRWPower = 0;
    private double prevRFWPower = 0;
    private double prevLFWPower = 0;

    public AutoPath autoPath;
    public SampleMecanumDrive rrdrive;

    private final ElapsedTime time = new ElapsedTime();

    public Drive(HardwareMap hardwareMap, RobotBase opMode) {
        this.hardwareMap = hardwareMap;
        this.robotBase = opMode;
        this.telemetry = robotBase.telemetry;
        this.util = new Util();

        rrdrive = new SampleMecanumDrive(this.hardwareMap);
        autoPath = new AutoPath(this.hardwareMap,this.robotBase);

        initHardware();
    }

    protected void initHardware() {
        leftFrontWheel  = hardwareMap.get(DcMotor.class, "LeftFrontWheel");
        leftFrontWheel.setDirection(DcMotor.Direction.REVERSE);
        leftFrontWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFrontWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftFrontWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFrontWheel.setPower(0.0);

        rightFrontWheel = hardwareMap.get(DcMotor.class, "RightFrontWheel");
        rightFrontWheel.setDirection(DcMotor.Direction.FORWARD);
        rightFrontWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFrontWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontWheel.setPower(0.0);

        leftRearWheel   = hardwareMap.get(DcMotor.class, "LeftRearWheel");
        leftRearWheel.setDirection(DcMotor.Direction.REVERSE);
        leftRearWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRearWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRearWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRearWheel.setPower(0.0);

        rightRearWheel  = hardwareMap.get(DcMotor.class, "RightRearWheel");
        rightRearWheel.setDirection(DcMotor.Direction.FORWARD);
        rightRearWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRearWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRearWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRearWheel.setPower(0.0);

        turtleMode = false;
    }

    /*
     * basicDrive takes inputs directly from gamepad and translates them to chassis movement.
     * This is sometimes called an arcade drive.
     */
    public void driveRobotCentric(double moveX, double moveY, double moveRotation) {

        double x  = util.roundTwo(moveX);
        double y  = -util.roundTwo(moveY);      // Remember, this is reversed!
        double rx = util.roundTwo(moveRotation);

        //Manage deadzone at center of joystick
        x  = (Math.abs(x) > Calibration.DEADZONE) ? x : 0;
        y  = (Math.abs(y) > Calibration.DEADZONE) ? y : 0;
        rx = (Math.abs(rx) > Calibration.DEADZONE) ? rx : 0;

        //telemetry.addData("moveX", x);
        //telemetry.addData("moveY", y);
        //telemetry.addData("moveRotation", rx);

        //Power factor is expiremental!
        double xTurnPowerFactor  = 1;
        double rxTurnPowerFactor = 0;
        if (Math.abs(x)>0 && Math.abs(rx)>0){
            xTurnPowerFactor  = 0.5;
            rxTurnPowerFactor = 0.5;
        } else if (Math.abs(rx)>0) {
            xTurnPowerFactor  = 0;
            rxTurnPowerFactor = 0.5;
        }

        double LFWPower = util.roundTwo(y + xTurnPowerFactor*x + rxTurnPowerFactor*rx);
        double LRWPower = util.roundTwo(y - xTurnPowerFactor*x + rxTurnPowerFactor*rx);
        double RFWPower = util.roundTwo(y - xTurnPowerFactor*x - rxTurnPowerFactor*rx);
        double RRWPower = util.roundTwo(y + xTurnPowerFactor*x - rxTurnPowerFactor*rx);

        //Facilitate quick turning while at speed
        if (rx<0 && RFWPower>0.8 && RRWPower>0.8 && LFWPower<0.1 && LRWPower<0.1) {
            LFWPower = -Math.abs(RRWPower);
            LRWPower = -Math.abs(RFWPower);
            // telemetry.addData("BD boosting LFWPower:", LFWPower);
            // telemetry.addData("BD boosting LRWPower:", LRWPower);
        }
        if (rx>0 && LFWPower>0.8 && LRWPower>0.8 && RFWPower<0.1 && RRWPower<0.1) {
            RFWPower = -Math.abs(LRWPower);
            RRWPower = -Math.abs(LFWPower);
            // telemetry.addData("BD boosting LFWPower:", RFWPower);
            // telemetry.addData("BD boosting LRWPower:", RRWPower);
        }

        // Send calculated power to wheels
        // telemetry.addData("BD sending RRWPower", RRWPower);
        // telemetry.addData("BD sending LRWPower", LRWPower);
        // telemetry.addData("BD sending RFWPower", RFWPower);
        // telemetry.addData("BD sending LFWPower", LFWPower);

        setPower(RRWPower, LRWPower, RFWPower, LFWPower);
    }

    public void setPower(double RRWPower, double LRWPower, double RFWPower, double LFWPower) {

        //Scale each motors input proportionally to be <=1
        double max = Collections.max(Arrays.asList(Math.abs(LFWPower), Math.abs(LRWPower), Math.abs(RFWPower), Math.abs(RRWPower), 1.0));
        if (max > 1.0) {
            // Divide everything by max (it's positive so we don't need to worry about signs)
            LFWPower /= max;
            LRWPower /= max;
            RFWPower /= max;
            RRWPower /= max;
        }

        //Scale inputs to maximum allowed power
        double maxPower = Calibration.MAX_DRIVE_POWER;
        if (turtleMode) maxPower = maxPower/2;
        RRWPower = util.roundTwo(Range.scale(RRWPower,0.0,1.0,0,maxPower));
        LRWPower = util.roundTwo(Range.scale(LRWPower,0.0,1.0,0,maxPower));
        RFWPower = util.roundTwo(Range.scale(RFWPower,0.0,1.0,0,maxPower));
        LFWPower = util.roundTwo(Range.scale(LFWPower,0.0,1.0,0,maxPower));

        double lastSetDrivePower = 0;
        if (RAMP_DRIVE_POWER && lastSetDrivePower >0.0 && false) {
            final double ticks = System.currentTimeMillis() - lastSetDrivePower;
            final double maxRamp = Math.max(1, RAMP_DRIVE_DURATION * ticks);

            RRWPower = util.roundTwo(ramp(prevRRWPower, RRWPower, maxRamp));
            prevRRWPower = RRWPower;

            LRWPower = util.roundTwo(ramp(prevLRWPower, LRWPower, maxRamp));
            prevLRWPower = LRWPower;

            RFWPower = util.roundTwo(ramp(prevRFWPower, RFWPower, maxRamp));
            prevRFWPower = RFWPower;

            LFWPower = util.roundTwo(ramp(prevLFWPower, LFWPower, maxRamp));
            prevLFWPower = LFWPower;
        }

//        telemetry.addData("setting RRWPower", RRWPower);
//        telemetry.addData("setting LRWPower", LRWPower);
//        telemetry.addData("setting RFWPower", RFWPower);
//        telemetry.addData("setting LFWPower", LFWPower);

        rightRearWheel.setPower(RRWPower);
        leftRearWheel.setPower(LRWPower);
        rightFrontWheel.setPower(RFWPower);
        leftFrontWheel.setPower(LFWPower);
    }

    protected double ramp(double oldVal, double newVal, double maxRamp) {
        double delta = newVal - oldVal;
        if (delta > maxRamp) {
            delta = maxRamp;
        } else if (delta < -maxRamp) {
            delta = -maxRamp;
        }
        return oldVal + delta;
    }

    public void stop() {
        leftFrontWheel.setPower(0.0);
        rightFrontWheel.setPower(0.0);
        leftRearWheel.setPower(0.0);
        rightRearWheel.setPower(0.0);
    }

    public void reset() {
        initHardware();
    }
}
