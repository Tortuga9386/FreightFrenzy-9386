package org.firstinspires.ftc.teamcode.drive.subsystems;

import com.arcrobotics.ftclib.controller.wpilibcontroller.SimpleMotorFeedforward;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.opmodes.RobotBase;

public class ShooterArcRobotics {
    protected HardwareMap hardwareMap;
    public Telemetry telemetry;
    public RobotBase robotBase;
    public MotorEx shooterFlywheel;
    public Servo triggerServo;
    private double  fullSpeed = 1.0;
    private double  targetVelocity = 2000.0;
    private boolean triggerLockout = false;
    private SimpleMotorFeedforward simpleMotorFeedforward;
    private SimpleMotorFeedforward flywheelFeedforward;
    private final ElapsedTime triggerMoveTimer = new ElapsedTime();
    private int triggerMoveMinimum = 300; //Measured in milliseconds

    public ShooterArcRobotics(HardwareMap hardwareMap, RobotBase opMode) {
        this.hardwareMap = hardwareMap;
        this.telemetry = opMode.telemetry;
        this.robotBase = opMode;
        initHardware();
    }

    protected void initHardware() {
        shooterFlywheel = new MotorEx(hardwareMap, "MotorOne", MotorEx.GoBILDA.BARE);
        shooterFlywheel.setInverted(true);
        shooterFlywheel.stopMotor();
        shooterFlywheel.resetEncoder();
        shooterFlywheel.setRunMode(MotorEx.RunMode.RawPower);
        //shooterFlywheel.setRunMode(Motor.RunMode.VelocityControl);
        //shooterFlywheel.setZeroPowerBehavior(MotorEx.ZeroPowerBehavior.FLOAT);
        //shooterMotorOne.setVeloCoefficients(1.489, 0.149, 0.0);
        //shooterFlywheel.setVeloCoefficients(0.05, 0.01, 0.31);
        //shooterFlywheel.setFeedforwardCoefficients(0.92, 0.47);

        simpleMotorFeedforward  = new SimpleMotorFeedforward(targetVelocity, targetVelocity/2);
        flywheelFeedforward     = new SimpleMotorFeedforward(.05, .01, 0.1);

        triggerServo = hardwareMap.get(Servo.class, "ShooterServo");
    }

    public void run(double speedMultiplier) {
        double newSpeed = fullSpeed*speedMultiplier;
        shooterFlywheel.set(newSpeed);
        callTheShot();
        telemetry.addData("shooterFlywheel.getVelocity()", shooterFlywheel.getVelocity());
    }

    public void runVelocity() {
        shooterFlywheel.set(flywheelFeedforward.calculate(450));
        telemetry.addData("shooterFlywheel.getVelocity()", shooterFlywheel.getVelocity());
    }

    //This method override allows user to send in no parameters, then feeds it into the base method.
    public void run() {
        run(fullSpeed);
    }

    public void update() {
        shooterFlywheel.set(simpleMotorFeedforward.calculate(targetVelocity));
    }

    public boolean isBusy() {
        if (shooterFlywheel.getVelocity() >= 0.0) {
            return true;
        }
        return false;
    }

    public void stop() {
        shooterFlywheel.set(0.0);
        shooterFlywheel.stopMotor();
    }

    public void reverse() {
        shooterFlywheel.set(-targetVelocity);
    }

    public void reset() {
        initHardware();
    }

    public void triggerForward() {
        triggerServo.setPosition(0.45);
    }

    private void triggerBack() {
        triggerServo.setPosition(0.0);
    }

    public void triggerRun() { triggerShoot(); }

    public void triggerStop() { triggerBack(); }

    public void triggerShoot() {
        //double triggerMoveStart = triggerMoveTimer.milliseconds();
        triggerForward();
        //double triggerMoveEnd = triggerMoveTimer.milliseconds();

    }

    public void callTheShot() {
        robotBase.intake.updateRingShooter();
    }

    public void triggerShootSafety() {
        double triggerMoveStart = triggerMoveTimer.milliseconds();
        triggerForward();
        double triggerMoveEnd = triggerMoveTimer.milliseconds();
        robotBase.intake.decrementRingCount();
        if (triggerMoveTimer.milliseconds() > triggerMoveMinimum) {
            triggerBack();
            triggerMoveTimer.reset();
        }
    }

    public void triggerShootAuto() {
        triggerForward();
        robotBase.intake.decrementRingCount();
        this.robotBase.util.sleep(300);
        triggerBack();
        this.robotBase.util.sleep(500);
    }

}