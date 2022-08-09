package org.firstinspires.ftc.teamcode.drive.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.opmodes.RobotBase;

public class Intake {
    protected HardwareMap hardwareMap;
    public Telemetry telemetry;
    private RobotBase robotBase;
    private DcMotor intakeMotorStage1;
    private DcMotor intakeMotorStage2;
    private DistanceSensor rampDistanceSensor;
    private DistanceSensor shooterDistanceSensor;
    private DigitalChannel[][] ledDisplay = new DigitalChannel[3][2];

    private final ElapsedTime intakeTimer = new ElapsedTime();
    private int ledToggleDelta = 400; //Measured in milliseconds

    private int ringCount = 0;

    private double rampCurrentDistance;
    private double ringOnRampDistance = 40; //Measured in mm
    private double shooterCurrentDistance;
    private double ringShooterDistance = 15; //Measured in mm
    private double fullSpeed = 1.0;

    boolean ringOnRamp = false;
    boolean ringFromShooter = false;

    public enum LedColor {
        RED,
        GREEN,
        AMBER,
        BLACK,
        OFF
    }

    public Intake(HardwareMap hardwareMap, RobotBase opMode) {
        this.hardwareMap = hardwareMap;
        this.robotBase = opMode;
        this.telemetry = robotBase.telemetry;
        initHardware();
    }

    protected void initHardware() {
        intakeMotorStage1 = hardwareMap.get(DcMotor.class, "IntakeOne");
        intakeMotorStage1.setDirection(DcMotor.Direction.FORWARD);
        intakeMotorStage1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeMotorStage1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intakeMotorStage1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        intakeMotorStage1.setPower(0.0);

        intakeMotorStage2 = hardwareMap.get(DcMotor.class, "IntakeTwo");
        intakeMotorStage2.setDirection(DcMotor.Direction.REVERSE);
        intakeMotorStage2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeMotorStage2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intakeMotorStage2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        intakeMotorStage2.setPower(0.0);

        rampDistanceSensor = hardwareMap.get(DistanceSensor.class, "ColorDistanceSensor");
        rampDistanceSensor.resetDeviceConfigurationForOpMode();
        shooterDistanceSensor = hardwareMap.get(DistanceSensor.class, "ShooterDistanceSensor");
        shooterDistanceSensor.resetDeviceConfigurationForOpMode();

        ledDisplay[0][0] = hardwareMap.get(DigitalChannel.class, "led1g");
        ledDisplay[0][0].setMode(DigitalChannel.Mode.OUTPUT);
        ledDisplay[0][1] = hardwareMap.get(DigitalChannel.class, "led1r");
        ledDisplay[0][1].setMode(DigitalChannel.Mode.OUTPUT);
        setLedDisplay(0, LedColor.GREEN);

        ledDisplay[1][0] = hardwareMap.get(DigitalChannel.class, "led2g");
        ledDisplay[1][0].setMode(DigitalChannel.Mode.OUTPUT);
        ledDisplay[1][1] = hardwareMap.get(DigitalChannel.class, "led2r");
        ledDisplay[1][1].setMode(DigitalChannel.Mode.OUTPUT);
        setLedDisplay(1, LedColor.RED);

        ledDisplay[2][0] = hardwareMap.get(DigitalChannel.class, "led3g");
        ledDisplay[2][0].setMode(DigitalChannel.Mode.OUTPUT);
        ledDisplay[2][1] = hardwareMap.get(DigitalChannel.class, "led3r");
        ledDisplay[2][1].setMode(DigitalChannel.Mode.OUTPUT);
        setLedDisplay(2, LedColor.GREEN);

    }

    public void run(double SpeedMultiplier) {
        double newSpeed = fullSpeed*SpeedMultiplier;
        intakeMotorStage1.setPower(newSpeed);
        intakeMotorStage2.setPower(newSpeed);
    }

    //This method override allows user to send in no parameters, then feeds it into the base method with the default of fullspeed.
    public void run() {
        run(fullSpeed);
    }

    public void runTrafficControl(double SpeedMultiplier) {
        double newSpeed = fullSpeed*SpeedMultiplier;
        updateRingOnRamp();
        if (ringCount >= 3) {
            intakeMotorStage1.setPower(0.0);
        } else if (ringOnRamp) {
            intakeMotorStage1.setPower(0.3);
            intakeMotorStage2.setPower(newSpeed);
        } else {
            run(SpeedMultiplier);
        }
    }

    //This method override allows user to send in no parameters, then feeds it into the base method with the default of fullspeed.
    public void runTrafficControl() {
        runTrafficControl(fullSpeed);
    }

    public boolean isBusy() {
        if (intakeMotorStage1.getPower() > 0.0 || intakeMotorStage2.getPower() > 0.0) {
            return true;
        }
        return false;
    }

    public void stop() {
        intakeMotorStage1.setPower(0.0);
        intakeMotorStage2.setPower(0.0);
    }

    public void reverse(double SpeedMultiplier) {
        double newSpeed = fullSpeed*SpeedMultiplier;
        intakeMotorStage1.setPower(newSpeed);
        intakeMotorStage2.setPower(newSpeed);
    }
    public void reverse() {
        reverse(-fullSpeed);
    }

    public void reverseTrafficControl(double SpeedMultiplier) {
        updateRingOnRamp();
        if (ringOnRamp) {
            decrementRingCount();
        }
        reverse(SpeedMultiplier);
    }
    public void reverseTrafficControl() {
        reverseTrafficControl(-fullSpeed);
    }

    public void getRampCurrentDistance() {
        rampCurrentDistance = rampDistanceSensor.getDistance(DistanceUnit.MM);
    }

    public void updateRingOnRamp() {
        //getRampCurrentDistance();
        telemetry.addData("Ramp Current Distance", rampCurrentDistance);
        telemetry.addData("Ring count", ringCount);

        if (rampCurrentDistance <= ringOnRampDistance) {
            if (ringOnRamp == false) {
                telemetry.addData("Detected new ring:", true);
                //incrementRingCount(); //Only increment if this is a new ring
            }
            telemetry.addData("Detected new ring:", "Still ring on ramp");
            ringOnRamp = true;
        } else {
            telemetry.addData("Detected new ring:", false);
            ringOnRamp = false;
        }
        setRingCountDisplay();
    }

    public void getShooterCurrentDistance() {
        shooterCurrentDistance = shooterDistanceSensor.getDistance(DistanceUnit.MM);
    }

    public void updateRingShooter() {
        getShooterCurrentDistance();
        telemetry.addData("Shooter Current Distance", rampCurrentDistance);
        telemetry.addData("Ring count", ringCount);

        if (shooterCurrentDistance <= ringShooterDistance) {
            if (ringFromShooter == false) {
                telemetry.addData("Detected new ring:", true);
                //decrementRingCount();
            }
            telemetry.addData("Detected new ring:", "Still ring on ramp");
            ringFromShooter = true;
        } else {
            telemetry.addData("Detected new ring:", false);
            ringFromShooter = false;
        }
        setRingCountDisplay();
    }

    public void incrementRingCount() {
        //ringCount++;
        if (ringCount > 4 && intakeMotorStage1.getPower()>=0.01) {
            //intakeMotorStage1.setPower(0.0);
        }
    }

    public void decrementRingCount() {
        if (ringCount > 0) {
            //ringCount--;
        }
    }

    public void resetRingCount() {
        ringCount = 0;
    }

    public void setRingCountDisplay() {
        switch (ringCount) {
            case 1:
                setLedDisplay(0, LedColor.GREEN);
                setLedDisplay(1, LedColor.RED);
                setLedDisplay(2, LedColor.RED);
                break;
            case 2:
                setLedDisplay(0, LedColor.GREEN);
                setLedDisplay(1, LedColor.GREEN);
                setLedDisplay(2, LedColor.RED);
                break;
            case 3:
                setLedDisplay(0, LedColor.GREEN);
                setLedDisplay(1, LedColor.GREEN);
                setLedDisplay(2, LedColor.GREEN);
                break;
            default:
                if (ringCount > 3 && intakeTimer.milliseconds() >= ledToggleDelta) {
                    setLedDisplay(0, LedColor.RED);
                    setLedDisplay(1, LedColor.RED);
                    setLedDisplay(2, LedColor.RED);
                    intakeTimer.reset();
                } else if (ringCount > 3) {
                    setLedDisplay(0, LedColor.GREEN);
                    setLedDisplay(1, LedColor.GREEN);
                    setLedDisplay(2, LedColor.GREEN);
                } else {
                    setLedDisplay(0, LedColor.RED);
                    setLedDisplay(1, LedColor.RED);
                    setLedDisplay(2, LedColor.RED);
                }
        }
    }

    private void setLedDisplay(int ledNumber, LedColor ledColor) {
        if (ledNumber>(ledDisplay.length)) { return; }
        DigitalChannel[] thisLed;
        thisLed = ledDisplay[ledNumber];
        switch (ledColor) {
            case GREEN:
                thisLed[0].setState(true);
                thisLed[1].setState(false);
                break;
            case RED:
                thisLed[0].setState(false);
                thisLed[1].setState(true);
                break;
            case AMBER:
                thisLed[0].setState(true);
                thisLed[1].setState(true);
                break;
            case BLACK:
            case OFF:
            default:
                thisLed[0].setState(false);
                thisLed[1].setState(false);
        }
    }

    public void reset() {
        initHardware();
    }

}