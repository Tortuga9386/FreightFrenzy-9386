package org.firstinspires.ftc.teamcode.drive.subsystems;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.drive.opmodes.MainAutonOp.StartPosition;

//This class stores persistant data when moving from TeleOp to AutoOp
public class GlobalVar {

    //startPosisition as selected by driver in autonomous
    private static StartPosition startPosisition;
    private static String allianceColor = "RED";

    //lastPose, this is a RoadRunner pose
    private static Pose2d lastPose = new Pose2d();

    //Setter method
    public static void setPose(Pose2d newPose){
        lastPose = newPose;
    }

    //Getter method
    public static Pose2d getPose(){
        return lastPose;
    }

    public static void setStartPosition(StartPosition robotStartPosition) {
        startPosisition = robotStartPosition;
        String positionToAllianceColor = startPosisition.toString().substring(0, 4).replace("_", "");
        if (!positionToAllianceColor.isEmpty()) {
            setAllianceColor(positionToAllianceColor);
        }
    }

    public static void setAllianceColor(String newAllianceColor) {
        allianceColor = newAllianceColor;
    }

    public static String getAllianceColor() {
        return allianceColor;
    }


}
