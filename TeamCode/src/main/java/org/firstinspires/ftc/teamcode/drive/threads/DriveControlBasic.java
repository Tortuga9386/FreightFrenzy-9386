package org.firstinspires.ftc.teamcode.drive.threads;

import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by MCHILEK on 12/21/2020.
 */
public class DriveControlBasic implements Runnable{
    //Mecanum drive wheels
    private DcMotor leftFrontWheel, leftRearWheel, rightFrontWheel, rightRearWheel;

    //Thead run condition
    private boolean isRunning = true;

    //Position variables used for storage and calculations
    // double verticalRightEncoderWheelPosition = 0, verticalLeftEncoderWheelPosition = 0, normalEncoderWheelPosition = 0,  changeInRobotOrientation = 0;
    // private double robotGlobalXCoordinatePosition = 0, robotGlobalYCoordinatePosition = 0, robotOrientationRadians = 0;
    // private double previousVerticalRightEncoderWheelPosition = 0, previousVerticalLeftEncoderWheelPosition = 0, prevNormalEncoderWheelPosition = 0;

    //Algorithm constants
    // private double robotEncoderWheelDistance;
    // private double horizontalEncoderTickPerDegreeOffset;

    //Sleep time interval (milliseconds) for the position update thread
    private int sleepTime;

    //Files to access the algorithm constants
    // private File wheelBaseSeparationFile = AppUtil.getInstance().getSettingsFile("wheelBaseSeparation.txt");
    // private File horizontalTickOffsetFile = AppUtil.getInstance().getSettingsFile("horizontalTickOffset.txt");

    // private int verticalLeftEncoderPositionMultiplier = 1;
    // private int verticalRightEncoderPositionMultiplier = 1;
    // private int normalEncoderPositionMultiplier = 1;

    /**
     * Constructor for GlobalCoordinatePosition Thread
     * @param verticalEncoderLeft left odometry encoder, facing the vertical direction
     * @param verticalEncoderRight right odometry encoder, facing the vertical direction
     * @param horizontalEncoder horizontal odometry encoder, perpendicular to the other two odometry encoder wheels
     * @param threadSleepDelay delay in milliseconds for the GlobalPositionUpdate thread (50-75 milliseconds is suggested)
     */
    // public DriveControl(DcMotor leftFrontWheel, DcMotor leftRearWheel, DcMotor rightFrontWheel, DcMotor rightRearWheel, int threadSleepDelay){
    //     this.leftFrontWheel  = leftFrontWheel;
    //     this.leftRearWheel   = leftRearWheel;
    //     this.rightFrontWheel = rightFrontWheel;
    //     this.rightRearWheel  = rightRearWheel;
    //     sleepTime = threadSleepDelay;


    // }

    /**
     * Updates the global (x, y, theta) coordinate position of the robot using the odometry encoders
     */
    // private void driveControlUpdate(){
    //     //Get Current Positions
    //     verticalLeftEncoderWheelPosition = (verticalEncoderLeft.getCurrentPosition() * verticalLeftEncoderPositionMultiplier);
    //     verticalRightEncoderWheelPosition = (verticalEncoderRight.getCurrentPosition() * verticalRightEncoderPositionMultiplier);

    //     double leftChange = verticalLeftEncoderWheelPosition - previousVerticalLeftEncoderWheelPosition;
    //     double rightChange = verticalRightEncoderWheelPosition - previousVerticalRightEncoderWheelPosition;

    //     //Calculate Angle
    //     changeInRobotOrientation = (leftChange - rightChange) / (robotEncoderWheelDistance);
    //     robotOrientationRadians = ((robotOrientationRadians + changeInRobotOrientation));

    //     //Get the components of the motion
    //     normalEncoderWheelPosition = (horizontalEncoder.getCurrentPosition()*normalEncoderPositionMultiplier);
    //     double rawHorizontalChange = normalEncoderWheelPosition - prevNormalEncoderWheelPosition;
    //     double horizontalChange = rawHorizontalChange - (changeInRobotOrientation*horizontalEncoderTickPerDegreeOffset);

    //     double p = ((rightChange + leftChange) / 2);
    //     double n = horizontalChange;

    //     //Calculate and update the position values
    //     robotGlobalXCoordinatePosition = robotGlobalXCoordinatePosition + (p*Math.sin(robotOrientationRadians) + n*Math.cos(robotOrientationRadians));
    //     robotGlobalYCoordinatePosition = robotGlobalYCoordinatePosition + (p*Math.cos(robotOrientationRadians) - n*Math.sin(robotOrientationRadians));

    //     previousVerticalLeftEncoderWheelPosition = verticalLeftEncoderWheelPosition;
    //     previousVerticalRightEncoderWheelPosition = verticalRightEncoderWheelPosition;
    //     prevNormalEncoderWheelPosition = normalEncoderWheelPosition;
    // }

    /**
     * Returns the robot's global x coordinate
     * @return global x coordinate
     */
    // public double returnXCoordinate(){ return robotGlobalXCoordinatePosition; }

    /**
     * Returns the robot's global y coordinate
     * @return global y coordinate
     */
    // public double returnYCoordinate(){ return robotGlobalYCoordinatePosition; }

    /**
     * Returns the robot's global orientation
     * @return global orientation, in degrees
     */
    // public double returnOrientation(){ return Math.toDegrees(robotOrientationRadians) % 360; }
    // public double returnOrientationDegrees(){ return Math.toDegrees(robotOrientationRadians) % 360; }
    // public double returnOrientationRadians(){ return robotOrientationRadians; }

    /**
     * Stops the drive control thread
     */
    public void stop(){ isRunning = false; }

    /**
     * Runs the thread
     */
    @Override
    public void run() {
        while(isRunning) {
            // driveControlUpdate();
            try {
                Thread.sleep(sleepTime);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
    }
}
