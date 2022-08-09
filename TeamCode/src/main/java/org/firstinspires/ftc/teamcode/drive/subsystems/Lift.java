package org.firstinspires.ftc.teamcode.drive.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.opmodes.MainAutonOp;
import org.firstinspires.ftc.teamcode.drive.opmodes.RobotBase;

public class Lift {
    protected HardwareMap hardwareMap;
    public Telemetry telemetry;
    private RobotBase robotBase;
    public DigitalChannel liftStopStore;
    public Arm arm;
    public Extender extender;
    public Claw claw;
    public boolean liftStopStoreSensor;


    public enum LiftPosition {
        CAPTURE,
        LEVEL_ONE,
        LEVEL_TWO,
        LEVEL_THREE,
        CAP
    }
    public LiftPosition liftTargetPosition = LiftPosition.CAPTURE;

    public Lift(HardwareMap hardwareMap, RobotBase opMode) {
        this.hardwareMap = hardwareMap;
        this.robotBase = opMode;
        this.telemetry = robotBase.telemetry;
        arm = new Arm(hardwareMap, opMode);
        extender = new Extender(hardwareMap, opMode);
        claw = new Claw(hardwareMap, opMode);
    }

    public void stop() {
        arm.stop();
        extender.stop();
        claw.open();
    }

    public class Arm {
        protected HardwareMap hardwareMap;
        public Telemetry telemetry;
        public DcMotor armMotor;
        public int targetTickPositionArm;
        double baseSpeed = 1.0;
        int minTicks = 40;
        int maxTicks = 2000;


        public Arm(HardwareMap hardwareMap, RobotBase opMode) {
            this.hardwareMap = hardwareMap;
            this.telemetry = opMode.telemetry;
            initHardware();
        }

        protected void initHardware() {
            armMotor = hardwareMap.get(DcMotor.class, "LiftArm");
            armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            armMotor.setDirection(DcMotor.Direction.REVERSE);
            armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            armMotor.setTargetPosition(0);
        }

        public void up() {
            setPositionByTicks(minTicks);
        }

        public void down() {
            setPositionByTicks(maxTicks);
        }

        public void setPositionByLevel(int targetLevel) {
            int targetTickPosition = 0;
            switch (targetLevel) {
                case 0:
                    targetTickPosition = 40;
                    break;
                case 1:
                    targetTickPosition = 240;
                    break;
                case 2:
                    targetTickPosition =  1100;
                    break;
                case 3:
                    targetTickPosition =  3000;
                    break;
                default:
                    targetTickPosition =  40;
            }

            telemetry.addData("Arm targetLevel", targetLevel);
            telemetry.addData("Arm targetTickPosition", targetTickPosition);
            setPositionByTicks(targetTickPosition);
        }


        public void setPositionByTicks(int targetTickPosition) {

            if (targetTickPosition < minTicks) {
                targetTickPosition = minTicks;
            } else if (targetTickPosition > maxTicks) {
                targetTickPosition = maxTicks;
            }

            targetTickPositionArm = targetTickPosition;
            armMotor.setTargetPosition(targetTickPosition);
            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            if (targetTickPosition < armMotor.getCurrentPosition()) {
                baseSpeed = -baseSpeed;
            }
            armMotor.setPower(baseSpeed);
        }

        public boolean isBusy() {
            return armMotor.isBusy();
        }

        public void stop() {
            setPositionByTicks(0);
            armMotor.setPower(0);
        }

    }

    public class Extender {

        protected HardwareMap hardwareMap;
        public Telemetry telemetry;
        public DcMotor extenderMotor;
        double baseSpeed = 0.4;
        int minTicks = 0;
        int maxTicks = 1100;

        public Extender(HardwareMap hardwareMap, RobotBase opMode) {
            this.hardwareMap = hardwareMap;
            this.telemetry = opMode.telemetry;
            initHardware();
        }


        protected void initHardware() {
            extenderMotor = hardwareMap.get(DcMotor.class, "LiftMotor");
            extenderMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            extenderMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            extenderMotor.setDirection(DcMotor.Direction.FORWARD);
            extenderMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            extenderMotor.setTargetPosition(0);
        }

        protected int getTicksForPosition(LiftPosition targetPosition) {
            telemetry.addData("Requesting ticks for targetPosition:", targetPosition.toString());
            switch (targetPosition) {
                case LEVEL_ONE:
                    return 75;
                case LEVEL_TWO:
                    return 600;
                case LEVEL_THREE:
                    return 1050;
                case CAP:
                    return 300;
                case CAPTURE:
                default:
                    return 0;
            }
        }

        public void setPositionByLevel(int targetLevel) {
            int targetTickPosition = 0;
            switch (targetLevel) {
                case 0:
                    targetTickPosition = 0;
                    break;
                case 1:
                    targetTickPosition = 350;
                    break;
                case 2:
                    targetTickPosition =  600;
                    break;
                case 3:
                    targetTickPosition =  1050;
                    break;
                default:
                    targetTickPosition =  0;
            }

            telemetry.addData("Arm Ext targetLevel", targetLevel);
            telemetry.addData("Arm Ext targetTickPosition", targetTickPosition);
            setPositionByTicks(targetTickPosition);
        }

        //Should only be used in an iterative opmode, will cause damage robot otherwise!
        public void calibrate() {
//            if (liftStopStoreState()) {
//                extenderMotor.setPower(0.0);
//                extenderMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            } else {
//                extenderMotor.setDirection(DcMotor.Direction.FORWARD);
//                extenderMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//                extenderMotor.setPower(0.0);
//            }
        }

        public void setStorePosition() {
            setPosition(LiftPosition.CAPTURE);
        }

        public void setPosition(LiftPosition targetPosition) {
            int ticksForPosition = getTicksForPosition(targetPosition);

            setPositionByTicks(ticksForPosition);
        }

        public void setPositionByTicks(int targetTickPosition) {
            double safteyTargetTickPosition = 0.0;
            int armHeightPosition = arm.armMotor.getCurrentPosition();

            if (targetTickPosition < minTicks) {
                targetTickPosition = minTicks;
            } else if (targetTickPosition > maxTicks) {
                targetTickPosition = maxTicks;
            }

            //Check here if we are going to crash the ground
//            if (armHeightPosition < 600) {
//                safteyTargetTickPosition = ((4.0/5.0) * armHeightPosition) + 100.0;
//                telemetry.addData("Arm Position for Ext adjust ", armHeightPosition);
//                telemetry.addData("Lift Extender Position Adjusted to ", safetyTargetTickPosition);
//                if (safteyTargetTickPosition < targetTickPosition) {
//                    targetTickPosition = Integer.valueOf((int) Math.round(safteyTargetTickPosition));
//                }
//            } else {
//                telemetry.addData("Lift Extender Position", "OK");
//            }

            extenderMotor.setTargetPosition(targetTickPosition);
            extenderMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            if (targetTickPosition < extenderMotor.getCurrentPosition()) {
                baseSpeed = -baseSpeed;
            }
            extenderMotor.setPower(baseSpeed);
        }

        public boolean isBusy() {
            return extenderMotor.isBusy();
        }

        public void stop() {
            extenderMotor.setPower(0);
        }

        public void reverse() {
            extenderMotor.setPower(-0.2);
        }

        public void reset() {
            initHardware();
        }
    }


    public class Claw {
        protected HardwareMap hardwareMap;
        public Telemetry telemetry;
        public Servo liftClawLeft;
        public Servo liftClawRight;

        public Claw(HardwareMap hardwareMap, RobotBase opMode) {
            this.hardwareMap = hardwareMap;
            this.telemetry = opMode.telemetry;
            initHardware();
        }

        protected void initHardware() {
            liftClawLeft = hardwareMap.get(Servo.class, "LiftClawLeft");
            liftClawLeft.resetDeviceConfigurationForOpMode();
            liftClawRight = hardwareMap.get(Servo.class, "LiftClawRight");
            liftClawRight.resetDeviceConfigurationForOpMode();
        }

        public void open() {
            liftClawLeft.setPosition(0.0);
            liftClawRight.setPosition(1.0);
//            liftClawLeft.setPosition(0.0);
//            liftClawRight.setPosition(0.6);
        }

        public void close() {
//            liftClawLeft.setPosition(1.0);
//            liftClawRight.setPosition(0.0);
            liftClawLeft.setPosition(1.0);
            liftClawRight.setPosition(0.0);
        }

        public void reset() {
            open();
        }
    }

}





