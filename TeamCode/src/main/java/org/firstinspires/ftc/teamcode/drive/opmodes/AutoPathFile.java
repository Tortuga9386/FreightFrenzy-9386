package org.firstinspires.ftc.teamcode.drive.opmodes;

//import com.fasterxml.jackson.*;
//import com.fasterxml.jackson.dataformat;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.opmodes.MainAutonOp.PathName;

import java.util.List;

/*
    Class to read autopaths direcltly from saved YAML files. This will require some time to understand and implement!
 */

public class AutoPathFile {
    //HardwareMap hardwareMap;
    Telemetry telemetry;
    SampleMecanumDrive mecanumDrive;
    public Pose2d startPose;
    public Trajectory driveToWobbleDrop;
    public Trajectory driveToShoot;
    public Trajectory driveToPark;

    public AutoPathFile(OpMode opMode) {
        telemetry = opMode.telemetry;

//        mapper = new ObjectMapper(new YAMLFactory());
//
//        //We need to use the findAndRegisterModules method so that Jackson will handle our Date properly:
//        mapper.findAndRegisterModules();
    }

    public class Path {
        public float x;
        float y;
        float heading;
        float startTangent;
        float resolution;
        float version;
        private List<Position> positions;

    }

    public class Position {
        float x;
        float y;
        float heading;
        float tangent;
        String interpolationType;
    }

    public void getAutoPaths(HardwareMap hardwareMap, PathName pathName) {
        mecanumDrive = new SampleMecanumDrive(hardwareMap);

//        Path path = mapper.readValue(new File("road-runner-quickstart-master/"+pathName+".yaml"), Path.class);


        telemetry.addData("DEFAULT paths for: ", pathName.toString());
        //If we get here, go nowhere to prevent the robot from hurting itself
        startPose = new Pose2d();
        mecanumDrive.setPoseEstimate(startPose);
        driveToWobbleDrop = mecanumDrive.trajectoryBuilder(startPose)
                .lineTo(new Vector2d(1, 1))
                .build();

        driveToShoot = mecanumDrive.trajectoryBuilder(driveToWobbleDrop.end())
                .lineTo(new Vector2d(-1, -1))
                .build();

        driveToPark = mecanumDrive.trajectoryBuilder(driveToShoot.end())
                .lineTo(new Vector2d(1, 1))
                .build();



    }

    public static void sleep(int milis){
        try {
            Thread.sleep(milis);
        } catch (Exception e){}
    }
}

