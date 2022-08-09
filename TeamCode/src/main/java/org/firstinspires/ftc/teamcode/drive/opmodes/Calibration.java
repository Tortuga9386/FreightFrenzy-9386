/**
 * Created by mchilek on 12/15/20
 */

package org.firstinspires.ftc.teamcode.drive.opmodes;

//import org.firstinspires.ftc.robotcore.external.Consumer;
//
//import java.io.File;
//import java.io.FileInputStream;
//import java.io.FileNotFoundException;
//import java.io.FileOutputStream;
//import java.io.IOException;
//import java.io.InputStream;
//import java.io.OutputStream;
//import java.io.OutputStreamWriter;
//import java.io.Writer;
//import java.lang.reflect.Field;
//import java.util.Properties;


public class Calibration extends ConstantsBase {

    //Start position
//    public static MainAutonOp.StartPosition START_POSITION = BLUE_LEFT;
    public static boolean DEBUG           = true;

    //Include external classes
//    public static boolean INITIALIZE_IMU = false;
//    public static boolean INITIALIZE_WEBCAM = false;
//    public static boolean INITIALIZE_LSA = false;

    //Webcam
    //public static int     TFOD_SAMPLES   = 5;

    //Joystick 
    public static double  DEADZONE              = 0.02;
    public static double  MIN_TOGGLE_TIME       = 0.3;

    //Drive
    public static double  MAX_DRIVE_POWER       = 0.8;

    //Wobble
//    public static double  WOBBLE_POS_UP   = 0.1;
//    public static double  WOBBLE_POS_DOWN = 0.9;

    //Intake 
//    public static double  FORWARD_INTAKE_SPEED = 0.7;
//    public static double  REVERSE_INTAKE_SPEED = 0.3;
//    public static boolean LOCK_INTAKE_WHEELS = true;

    //Shooter 
//    public static double SHOOTER_POWER   = 0.05;

    //Lights 
    public static int    BLINKIN_UPDATE_LOCKOUT = 150;

    public Calibration() {}
}
