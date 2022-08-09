package org.firstinspires.ftc.teamcode.drive.opmodes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import static org.firstinspires.ftc.teamcode.drive.opmodes.MainAutonOp.State.*;
import static org.firstinspires.ftc.teamcode.drive.subsystems.GlobalVar.setPose;
import static org.firstinspires.ftc.teamcode.drive.subsystems.GlobalVar.setStartPosition;

//@Autonomous(name="Autonomous", group = "auto")
@Disabled
public class MainAutonOp extends RobotBase {

    private boolean GET_SECOND_WOBBLE = false;
    private boolean GET_RING_STACK = true;

    public enum State {
        BEGIN,
        READ_WEBCAM,
        GET_AUTO_PATHS,
        SECOND_POSITION_DELAY,
        DRIVE_TO_SHIPPING_HUB,
        DROP_FREIGHT,
        DRIVE_TO_CAROUSEL,
        DELIVER_DUCK,
        DRIVE_TO_PARK,
        PARK,
        IDLE
    }
    public State currentState = BEGIN;

    public enum StartPosition {
        NONE,
        BLUE_LEFT,
        BLUE_RIGHT,
        RED_LEFT,
        RED_RIGHT,
        RED_LEFT_INSTA_PARK
    }
    public StartPosition startPosition = StartPosition.NONE;

    public enum PathName {
        BLUE_LEFT,
        BLUE_RIGHT,
        RED_LEFT,
        RED_RIGHT
    }
    public PathName pathName;
    public AutoPath autoPath;
    public SampleMecanumDrive rrdrive;

    //Timers
    private final ElapsedTime time = new ElapsedTime();
    private long iterationCounter = 0;
    double elapsedTime = 0;
    double secondPositionTime = 0;
    double clawStartTime = 0;
    double carouselStartTime = 0;
    double parkTime = 5.0;
    ElapsedTime parkTimer = new ElapsedTime();

    int triggerShootCount = 0;
    /**
     * Constructor
     */
    public MainAutonOp() {
    }

    @Override
    public void init() {
        super.INITIALIZE_WEBCAM     = true;
        super.INITIALIZE_DRIVE      = false;
        super.init();
        rrdrive = new SampleMecanumDrive(hardwareMap);
        autoPath = new AutoPath(hardwareMap,this);

        setStartPosition(startPosition); //This is the enum startPosition above, which will be passed to TeleOp
        telemetry.addData("Start Position", startPosition.toString());
        telemetry.addData("State", "READY");
        currentState = BEGIN;
        iterationCounter = 1;

        lift.claw.close();
        odometryWheelLift.down();

        //AutoTransitioner.transitionOnStop(this, "TeleOp");
    }

    @Override
    public void init_loop() {
        super.init_loop();
//        webcam.getObjectDetection();
//        telemetry.addData("Webcam object label", webcam.objectLabel);
//        telemetry.addData("Webcam object left", webcam.objectLeft);
//        telemetry.addData("Webcam object top", webcam.objectTop);
//        telemetry.addData("Webcam object right", webcam.objectRight);
//        telemetry.addData("Webcam object bottom", webcam.objectBottom);
//        telemetry.addData("TeleOp Barcode Location", webcam.barcodeLocation);

        //imu_loop();
    }

    private void imu_loop() {
        if (INITIALIZE_IMU && controlHub.initialized) {
            Orientation allAngles = controlHub.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            double      globalAngle = allAngles.firstAngle;
            telemetry.addData("IMU angle:", " (%.2f)", globalAngle);
        }
    }

    protected State handleState(State state) {

        switch (state) {
            case BEGIN:
                return READ_WEBCAM;
                //return DRIVE_TO_PARK;

            case READ_WEBCAM:
                webcam.getObjectDetection();
                telemetry.addData("Webcam object label", webcam.objectLabel);
                telemetry.addData("Webcam object left", webcam.objectLeft);
                telemetry.addData("Webcam object top", webcam.objectTop);
                telemetry.addData("Webcam object right", webcam.objectRight);
                telemetry.addData("Webcam object bottom", webcam.objectBottom);
                telemetry.addData("TeleOp Barcode Location", webcam.barcodeLocation);

                //webcam.getVuforiaLocalization(); //Targets are too far away, use telephoto lens? TODO
                //webcam.getTensorFlowObjectDetection(); //Field of view is somehow too small, investigate later? TODO
                //webcam.getTensorFlowObjectDetectionCurrentGame();
                return GET_AUTO_PATHS;
              
            case GET_AUTO_PATHS:
                String barcodeLocation = "LEFT";
                if (!webcam.barcodeLocation.isEmpty()) { barcodeLocation = webcam.barcodeLocation.toUpperCase(); }
                switch (barcodeLocation) {
                    case "RIGHT":
                        autoPath.shippingHubLevel = 3;
                        break;
                    case "CENTER":
                        autoPath.shippingHubLevel = 2;
                        break;
                    case "LEFT":
                    default:
                        autoPath.shippingHubLevel = 1;
                        break;
                }
                telemetry.addData("webcam.barcodeLocation", webcam.barcodeLocation);
                telemetry.addData("barcodeLocation", barcodeLocation);
                telemetry.addData("autoPath.shippingHubLevel", autoPath.shippingHubLevel);

                String pathNameAsString = startPosition.toString(); //This is not needed in simplified paths // +"_"+ fieldObjectsLabel;
                if (pathNameAsString != null && !pathNameAsString.trim().isEmpty()) {
                    pathName = PathName.valueOf(pathNameAsString);
                }

                autoPath.getAutoPaths(pathName);

                if (pathName == PathName.RED_RIGHT || pathName == PathName.BLUE_LEFT) {
                    return SECOND_POSITION_DELAY;
                }
                return DRIVE_TO_SHIPPING_HUB;

            case SECOND_POSITION_DELAY:
                if (secondPositionTime == 0) { secondPositionTime = elapsedTime; }
                if (secondPositionTime+10.0 < elapsedTime) {
                    return DRIVE_TO_SHIPPING_HUB;
                }
                break;

            case DRIVE_TO_SHIPPING_HUB:
                //RoadRunner sets freight drop position on the way
                //lift.arm.setPositionByLevel(autoPath.shippingHubLevel);
                //lift.extender.setPositionByLevel(autoPath.shippingHubLevel);

                if (!rrdrive.isBusy()) {
                    telemetry.addData("RRPath", "driveToShippingHub (" + time.seconds()+ ")");
                    try {
                        rrdrive.setPoseEstimate(autoPath.startPose);
                        rrdrive.followTrajectory(autoPath.driveToShippingHub);
                    } catch (Exception e) {
                        telemetry.addData("Error running ", pathName.toString() + " driveToShippingHub");
                        telemetry.addData("Exception Message: ",e.toString());
                        return IDLE;
                    }
                    //return IDLE;
                    return DROP_FREIGHT;
                }
                break;

            case DROP_FREIGHT:
                //Make sure claw is open long enough to drop the freight
                lift.claw.open();

                if (!rrdrive.isBusy()) {
                    if (clawStartTime == 0) { clawStartTime = elapsedTime; }
                    if (clawStartTime+2.0 < elapsedTime) {
                        telemetry.addData("lift.claw.open()", iterationCounter);

                        parkTimer.reset();
                        return DRIVE_TO_CAROUSEL;
                    } else {
                        telemetry.addData("Claw start", carouselStartTime);
                        telemetry.addData("Waiting for claw", elapsedTime);
                        return DROP_FREIGHT;
                    }
                }
                break;

            case DRIVE_TO_CAROUSEL:
                //RoadRunner stores lift
                //lift.extender.setPositionByLevel(0);
                //lift.arm.setPositionByLevel(0);

                if (!rrdrive.isBusy()) {
                    telemetry.addData("RRPath", "driveToCarousel (" + time.seconds()+ ")");
                    try {
                        rrdrive.followTrajectory(autoPath.driveToCarousel);
                    } catch (Exception e) {
                        telemetry.addData("Error running ", pathName.toString() + " driveToCarousel");
                        telemetry.addData("Exception Message: ", e.toString());
                        return IDLE;
                    }

                    return DELIVER_DUCK;
                }
                break;

            case DELIVER_DUCK:
                //Make sure we get to the carousel before we start, then give it time to run
                if (!rrdrive.isBusy()) {

                    //Skip this stage for warehouse parking
                    if (pathName == PathName.RED_RIGHT || pathName == PathName.BLUE_LEFT) {
                        parkTimer.reset();
                        return DRIVE_TO_PARK;
                    }

                    if (pathName == PathName.BLUE_RIGHT) {
                        carouselDrive.runBlue();
                    }else{
                        carouselDrive.run();
                    }

                    if (carouselStartTime == 0) { carouselStartTime = elapsedTime; }
                    if (carouselStartTime+2.2 < elapsedTime) {
                        telemetry.addData("carouselDrive.run()", iterationCounter);

                        parkTimer.reset();
                        return DRIVE_TO_PARK;
                    } else {
                        telemetry.addData("Carousel start", carouselStartTime);
                        telemetry.addData("Waiting for carousel run ", elapsedTime);
                        return DELIVER_DUCK;
                    }
                }
                break;

            case DRIVE_TO_PARK:
                if (!rrdrive.isBusy()) {
                    telemetry.addData("RRPath", "driveToPark (" + time.seconds() + ")");
                    try {
                        rrdrive.followTrajectory(autoPath.driveToPark);
                    } catch (Exception e) {
                        telemetry.addData("Error running ", pathName.toString() + " driveToPark");
                        telemetry.addData("Exception Message: ", e.toString());
                        return IDLE;
                    }
                    return PARK;
                }
                break;

            case PARK:
                if (rrdrive.isBusy()) break;
                if (parkTimer.seconds() >= parkTime) {
                    carouselDrive.stop();
                    return IDLE;
                } else {
                    //This happens while the robot is parked.
                }
                break;

            default:
                break;
        }
        return state;
    }

    @Override
    public void loop() {
        double elapsed = time.seconds();
        elapsedTime = time.seconds();

        telemetry.setAutoClear(false);
        telemetry.addData("timer: ", elapsed);
        iterationCounter += 1;
        if (currentState == IDLE) {
            telemetry.addData("State:", "" + currentState + " / " + iterationCounter);
            telemetry.update();
        } else {
            telemetry.addData("State:", "pre " + currentState + " / " + iterationCounter);
            telemetry.update();
            State new_state = handleState(currentState);
            telemetry.addData("State:", "post " + currentState + " / " + iterationCounter);
            telemetry.update();
            if (new_state != currentState) {
                time.reset();
                currentState = new_state;
            }
        }

        //Update drive continuously in the background, regardless of state
        rrdrive.update();
        setPose(rrdrive.getPoseEstimate()); //This will be passed to TeleOp

        // Read current pose
        Pose2d poseEstimate = rrdrive.getPoseEstimate();

        // Print pose to telemetry
        telemetry.addData("x", poseEstimate.getX());
        telemetry.addData("y", poseEstimate.getY());
        telemetry.addData("heading", poseEstimate.getHeading());
        telemetry.update();
    }

    public void delay() {

    }

    @Override
    public void stop() {
        super.stop();
    }
    

}