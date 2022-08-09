package org.firstinspires.ftc.teamcode.drive.opmodes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.drive.StandardTrackingWheelLocalizer;
import org.firstinspires.ftc.teamcode.drive.subsystems.Lift;
import org.firstinspires.ftc.teamcode.drive.subsystems.LocalGamepad;
import org.firstinspires.ftc.teamcode.drive.subsystems.OdometryWheelLift;

import java.util.Arrays;

import static org.firstinspires.ftc.teamcode.drive.subsystems.GlobalVar.getPose;
import static org.firstinspires.ftc.teamcode.drive.subsystems.GlobalVar.setPose;

@TeleOp(name="***TeleOp***", group="teleop")
public class MainTeleOp extends RobotBase
{
    private   ElapsedTime   runtime = new ElapsedTime();
    private   double        lastArmLiftUpdateTime = 0.0;
    private   LocalGamepad  localGamepad1, localGamepad2;
    private   double        turtleToggleUpdate = 0.0;
    protected double        globalAngle = 0.0;

    //Keep track of where we are, this is necessary for the alignToShoot function
    StandardTrackingWheelLocalizer rrLocalizer;

    public MainTeleOp() {}

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        super.INITIALIZE_WEBCAM     = false;
        super.init();
        localGamepad1 = new LocalGamepad(gamepad1);
        localGamepad2 = new LocalGamepad(gamepad2);
        rrLocalizer   = new StandardTrackingWheelLocalizer(hardwareMap);

        //Set your initial pose from position stored from AutoOp
        rrLocalizer.setPoseEstimate(getPose());
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        telemetry.addData("Status", "Run Time:" + runtime.toString());
        localGamepad1.update();
        localGamepad2.update();

        drive_loop();
        carousel_drive_loop();
        webcam_loop();
        lift_loop();
        odometry_lift_loop();
        //light_loop();
        //imu_loop();
        telemetry_loop();
    }

    protected void drive_loop() {

        if (gamepad1.right_bumper && false) {
            if (gamepad1.y){
                drive.leftFrontWheel.setPower(1.0);
            } else if (gamepad1.b) {
                drive.rightFrontWheel.setPower(1.0);
            } else if (gamepad1.a) {
                drive.rightRearWheel.setPower(1.0);
            } else if (gamepad1.x) {
                drive.leftRearWheel.setPower(1.0);
            } else {
                drive.stop();
            }
        }

        if (turtleToggleUpdate + Calibration.MIN_TOGGLE_TIME<=runtime.seconds()) {
            if (gamepad1.a && !this.turtleMode){
                updateTurtleMode(true);
                turtleToggleUpdate = runtime.seconds();
            } else if (gamepad1.a && this.turtleMode) {
                updateTurtleMode(false);
                turtleToggleUpdate = runtime.seconds();
            }
        }

        //telemetry.addData("Turtle Mode", this.turtleMode);

        // Make sure to call myLocalizer.update() on *every* loop
        // Increasing loop time by utilizing bulk reads and minimizing writes will increase your odometry accuracy
        rrLocalizer.update();
        setPose(rrLocalizer.getPoseEstimate()); //Store pose for future interactions

        //Use localGamepad1.XasCircle and localGamepad1.YasCircle to prevent skewing in corners of joystick square
        //telemetry.addData("Drive Mode", driveMode.toString());
        switch (driveMode) {
            case ARC_DRIVE_ROBOT:
                driveArcRobotics.driveRobotCentric(localGamepad1.XasCircle, localGamepad1.YasCircle, gamepad1.right_stick_x);
                break;
            case ARC_DRIVE_FIELD:
                driveArcRobotics.driveFieldCentric(localGamepad1.XasCircle, localGamepad1.YasCircle, gamepad1.right_stick_x, globalAngle);
                break;
            case ORIGINAL_CRISPY:
            default:
                drive.driveRobotCentric(localGamepad1.XasCircle, localGamepad1.YasCircle, gamepad1.right_stick_x);
        }
        if (false) { //button combination to rotate through drive modes, really shouldnt be doing this in a match!
            //driveMode = getNextDriveMode(driveMode);
        }
    }
    public void updateTurtleMode(boolean newTurtleMode) {
        this.turtleMode = newTurtleMode;
        if (drive != null)    { drive.turtleMode  = newTurtleMode; }
        if (driveArcRobotics != null) { driveArcRobotics.turtleMode = newTurtleMode; }
    }
    public DriveMode getNextDriveMode(DriveMode oldDriveMode) {
        int index = oldDriveMode.ordinal();
        int nextIndex = index + 1;
        DriveMode[] driveModes = DriveMode.values();
        nextIndex %= driveModes.length;
        return driveModes[nextIndex];
    }

    protected void carousel_drive_loop() {
        if (carouselDrive == null) {
            return;
        }
        //Add buttons to activate carousel drive here
        if (gamepad1.y == true) {
            carouselDrive.run();
        } else {
            carouselDrive.stop();
        }
    }


    protected void webcam_loop() {
        if (webcam == null) {
            return;
        }
        //Activate flywheel and shooter servo based on button combos
        //if (gamepad1.right_trigger == 1){
            webcam.getObjectDetection();
            telemetry.addData("Webcam object label", webcam.objectLabel);
            telemetry.addData("Webcam object left", webcam.objectLeft);
            telemetry.addData("Webcam object top", webcam.objectTop);
            telemetry.addData("Webcam object right", webcam.objectRight);
            telemetry.addData("Webcam object bottom", webcam.objectBottom);
            telemetry.addData("TeleOp Barcode Location", webcam.barcodeLocation);
        //}
    }

    protected void lift_loop() {
        //Run lift and lift claw based on button combos
        if (lift == null) {
            return;
        }

        //Lift.Arm up and down, only allow updates every ?? seconds
        int currentArmPosition = lift.arm.armMotor.getCurrentPosition();
        //if (runtime.time() - lastArmLiftUpdateTime > 0.0001) {
            if (gamepad1.left_bumper) {
                //telemetry.addData("Lift Arm Action", "lift.arm.down;");
                lift.arm.setPositionByTicks(currentArmPosition -= 65);//Best postion so far 75
            } else if (gamepad1.right_bumper) {
                //telemetry.addData("Lift Arm Action", "lift.arm.up;");
                lift.arm.setPositionByTicks(currentArmPosition += 65);//Best postion so far 75
            } else {
                //telemetry.addData("Lift Arm Action", "None");
            }
            lastArmLiftUpdateTime = runtime.time();
        //}
        telemetry.addData("Arm Lift Position", currentArmPosition);

        //Lift.Extender in and out
        int currentExtenderPosition = lift.extender.extenderMotor.getCurrentPosition();
        if (gamepad1.left_trigger > 0.1){
            //telemetry.addData("Lift Ext Action", "lift.extender.setPositionByTicks(in);");
            lift.extender.setPositionByTicks(currentExtenderPosition+=100);
        } else if (gamepad1.right_trigger > 0.1) {
            //telemetry.addData("Lift Ext Action", "lift.extender.setPositionByTicks(out);");
            lift.extender.setPositionByTicks(currentExtenderPosition-=100);
        } else {
            //lift.extender.setPositionByTicks(currentExtenderPosition);
            //telemetry.addData("Lift Ext Action", "None");
        }
        telemetry.addData("Arm Extender Position", currentExtenderPosition);

        //Lift.Claw open and close
        if (gamepad1.b) {
            lift.claw.open();
            //telemetry.addData("Claw Servo", "OPEN");
        } else {
            lift.claw.close();
            //telemetry.addData("Claw Servo", "CLOSE");
        }
    }

    protected void odometry_lift_loop() {
        if (odometryWheelLift == null) {
            return;
        }
        //Add buttons to activate carousel drive here
        if (gamepad1.x) {
            odometryWheelLift.down();
        } else {
            odometryWheelLift.up();
        }
    }

    protected void light_loop() {
        if (lights == null) {
            return;
        }

        String lightPattern;
        if (gamepad1.x){
            lightPattern = "BLUE";
        } else if (gamepad1.a) {
            lightPattern = "GREEN";
        } else if (gamepad1.y) {
            lightPattern = "ORANGE";
        } else if (gamepad1.b) {
            lightPattern = "RED";
        } else {
            lightPattern = "BLACK";
        }

        lights.setPattern(lightPattern);
    }

    protected void imu_loop() {
        if (INITIALIZE_IMU && controlHub!=null && controlHub.initialized) {
            Orientation allAngles = controlHub.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            globalAngle = allAngles.firstAngle;
        }
    }

    protected void telemetry_loop() {

        if (rrLocalizer!= null) {
            Pose2d currentPose = rrLocalizer.getPoseEstimate();
            telemetry.addData("Encoder values (L, R, F) ", Arrays.toString(rrLocalizer.getWheelPositions().toArray()));
            telemetry.addData("x", currentPose.getX());
            telemetry.addData("y", currentPose.getY());
            telemetry.addData("heading radians", currentPose.getHeading());
            telemetry.addData("heading degrees", Math.toDegrees(currentPose.getHeading()));
        }

        if (INITIALIZE_IMU && controlHub!=null && controlHub.initialized) {
            telemetry.addData("IMU angle:", " (%.2f)", globalAngle);
        }
    }

}
