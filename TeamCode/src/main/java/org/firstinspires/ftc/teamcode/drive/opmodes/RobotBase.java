package org.firstinspires.ftc.teamcode.drive.opmodes;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.drive.subsystems.*;


import java.util.List;

/**
 * This is NOT an opmode.
 *
 * This class initiates all the specific hardware for a robot.
 * The various subsystems classes are located in the subsystem folder.
 * This code is used by all the other opmodes.
 *
 */
@Disabled
public class RobotBase extends OpMode
{
    //Define constants that should NOT be adjusted by manual calibration
    public static double COUNTS_PER_INCH        = 1892.3686435854;
    protected boolean INITIALIZE_WEBCAM         = false;
    protected boolean INITIALIZE_IMU            = false;
    protected boolean INITIALIZE_DRIVE          = true;
    protected boolean INITIALIZE_LSA            = false;
    public    boolean turtleMode                = false;

    public enum DriveMode {
        ORIGINAL_CRISPY,
        ARC_DRIVE_ROBOT,
        ARC_DRIVE_FIELD
    }
    public DriveMode driveMode = DriveMode.ORIGINAL_CRISPY;

    //Make subsystems available to all class extensions
    //public MenuController menu_controller;
    public Drive drive;
    public DriveArcRobotics driveArcRobotics;
    public CarouselDrive carouselDrive;

    public Intake intake;
    public Shooter shooter;
    public OdometryWheelLift odometryWheelLift;
    public ShooterArcRobotics shooterArcRobotics;

    public Lift lift;
    public Lights lights;
    public Webcam webcam;
    public ControlHub controlHub;
    public LocalizationSA localizationSA;
    public Util util;

    /* Constructor */
    public RobotBase(){ }

    /* Initialize standard Hardware interfaces */
    @Override
    public void init() {
        //Read calibration constants
        //new Calibration().readFromFile();

        //1 of 2 >> Initialize sensor bulk read, then after all the hardware maps are set below, enable BulkCachingMode.AUTO
        List<LynxModule> hubs = hardwareMap.getAll(LynxModule.class);

        //Initialize subsystems
        //menu_controller = new MenuController(new Calibration());


        carouselDrive = new CarouselDrive(hardwareMap, this);
        lift = new Lift(hardwareMap, this);
        odometryWheelLift = new OdometryWheelLift(hardwareMap, this);
        //util = new Util();
        //lights = new Lights(hardwareMap, this);
        //lights.setPattern("GREEN");


        //Enable IMU for TFOD
        if (INITIALIZE_DRIVE) {
            drive = new Drive(hardwareMap, this);
        }

        //Enable IMU for TFOD
        if (INITIALIZE_WEBCAM) {
            webcam = new Webcam(hardwareMap, this);
        }

        //Enable IMU for angles
        if (INITIALIZE_IMU) {
            controlHub = new ControlHub(hardwareMap, this);
        }

        //Enable Stand Alone Localization (NOT ROADRUNNER)
        if (INITIALIZE_LSA) {
            localizationSA = new LocalizationSA(hardwareMap, this);
        }

        //2 of 2 >> All the hardware maps are set above, now enable BulkCachingMode.AUTO on hubs
        for (LynxModule hub : hubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
        telemetry.clearAll();
    }

    @Override
    public void loop() {
        //Do nothing, use these classes in the opModes
    }

    /*
     * Code to run when the op mode is first disabled goes here
     */
    @Override
    public void stop() {

        if (intake != null) {
            intake.stop();
        }
        if (shooter != null) {
            shooter.stop();
        }
        if (lift != null) {
            lift.stop();
        }
        if (lights != null) {
            lights.stop();
        }
        if (drive != null) {
            drive.stop();
        }
        if (driveArcRobotics != null) {
            driveArcRobotics.stop();
        }
        if (webcam != null && webcam.initialized) {
            webcam.stop();
        }
        if (controlHub != null && controlHub.initialized) {
            controlHub.stop();
        }
        if (localizationSA != null && localizationSA.initialized) {
            localizationSA.stop();
        }

    }
    
 }
