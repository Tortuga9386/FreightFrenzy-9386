package org.firstinspires.ftc.teamcode.drive.opmodes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.opmodes.MainAutonOp.PathName;

import java.util.Arrays;

import static org.firstinspires.ftc.teamcode.drive.subsystems.GlobalVar.getPose;

public class AutoPath {
    private HardwareMap hardwareMap;
    private RobotBase robotBase;
    private Telemetry telemetry;
    private SampleMecanumDrive mecanumDrive;

    public Pose2d startPose;
    public Integer shippingHubLevel = 1;
    public Trajectory driveToShippingHub; //DRIVE_TO_SHIPPING_HUB
    public Trajectory driveToCarousel;    //DRIVE_TO_CAROUSEL
    public Trajectory driveToPark;        //DRIVE_TO_PARK

    public AutoPath(HardwareMap hardwareMap, RobotBase opMode) {
        this.hardwareMap = hardwareMap;
        this.telemetry = opMode.telemetry;
        this.robotBase = opMode;
    }
    public void getAutoPaths(PathName pathName) {
        mecanumDrive = new SampleMecanumDrive(hardwareMap);

        /*
            All these cases should be handled!
            BLUE_LEFT,BLUE_RIGHT_QUAD,
            RED_LEFT,RED_RIGHT
        */
        switch (pathName){

            /***************************************************************************

             RED LEFT INSTANT PARK

             ***************************************************************************/

//            case RED_LEFT_INSTA_PARK:
//                startPose = new Pose2d(-38.0,-60.0,0.0);
//                mecanumDrive.setPoseEstimate(startPose);
//                driveToShippingHub = mecanumDrive.trajectoryBuilder(startPose)
//                        .lineToConstantHeading(new Vector2d(-59.0, -36.0))
//                        .build();
//                break;

            /***************************************************************************

             BLUE LEFT

             ***************************************************************************/
            case BLUE_LEFT:
                startPose = new Pose2d(12,63.5,Math.toRadians(270));
                mecanumDrive.setPoseEstimate(startPose);
                driveToShippingHub = mecanumDrive.trajectoryBuilder(startPose)
                        //First add the path segments to follow
                        .splineTo(new Vector2d(-11.0, 42.0), Math.toRadians(270))

                        //Next add the activities during the path (THESE ARE TIME BASED!)
                        .addTemporalMarker(0.0, () -> {
                            //Raise the arm to drop freight at correct height
                            robotBase.lift.arm.setPositionByLevel(shippingHubLevel);
                            robotBase.lift.extender.setPositionByLevel(shippingHubLevel);
                        })
                        .build();

                driveToCarousel = mecanumDrive.trajectoryBuilder(driveToShippingHub.end())
                        //First add the path segments to follow
                        .lineToLinearHeading(new Pose2d(10, 60, Math.toRadians(0)))

                        //Next add the activities during the path (THESE ARE TIME BASED!)
                        .addTemporalMarker(0.5, () -> {
                            robotBase.lift.extender.setPositionByLevel(0);

                        })
                        .addTemporalMarker(1.5, () -> {
                            //Retract the lift on the way
                            robotBase.lift.claw.close();
                            robotBase.lift.arm.setPositionByLevel(0);
                        })

                        .build();

                driveToPark = mecanumDrive.trajectoryBuilder(driveToCarousel.end())
                        .lineToLinearHeading(new Pose2d(36, 60, Math.toRadians(0)))
                        .addTemporalMarker(0, () -> {
                            //robotBase.odometryWheelLift.up();
                        })
                        .build();
            break;

                /***************************************************************************

                 BLUE RIGHT

                 ***************************************************************************/
//////


            case BLUE_RIGHT:
                startPose = new Pose2d(-36,63.5,Math.toRadians(270));
                mecanumDrive.setPoseEstimate(startPose);
                driveToShippingHub = mecanumDrive.trajectoryBuilder(startPose)
                        .splineTo(new Vector2d(-11.0, 42.0), Math.toRadians(270))

                        //Next add the activities during the path (THESE ARE TIME BASED!)
                        .addTemporalMarker(0.0, () -> {
                            //Raise the arm to drop freight at correct height
                            robotBase.lift.arm.setPositionByLevel(shippingHubLevel);
                            robotBase.lift.extender.setPositionByLevel(shippingHubLevel);
                        })
                        .build();

                driveToCarousel = mecanumDrive.trajectoryBuilder(driveToShippingHub.end())
                        .lineToLinearHeading(new Pose2d(-64, 54, Math.toRadians(270)))

                        .addTemporalMarker(1.0, () -> {
                            robotBase.lift.extender.setPositionByLevel(0);

                        })
                        .addTemporalMarker(2.0, () -> {
                            //Retract the lift on the way
                            robotBase.lift.claw.close();
                            robotBase.lift.arm.setPositionByLevel(0);
                            robotBase.carouselDrive.runBlue();
                        })
                        .build();

                driveToPark = mecanumDrive.trajectoryBuilder(driveToCarousel.end())
                        .lineToLinearHeading(new Pose2d(-60, 35, Math.toRadians(270)))

                        .addTemporalMarker(0, () -> {
                            //Stop any activity on way to park
                            robotBase.carouselDrive.stop();
                        })
                        .build();
                break;



                /***************************************************************************

                 RED LEFT

                 ***************************************************************************/
/////
            case RED_LEFT:
                startPose = new Pose2d(-36.0,-63.5,Math.toRadians(90));
                mecanumDrive.setPoseEstimate(startPose);
                driveToShippingHub = mecanumDrive.trajectoryBuilder(startPose)
                        .splineTo(new Vector2d(-11.0, -42.0), Math.toRadians(90)) // keep

                        //Next add the activities during the path (THESE ARE TIME BASED!)
                        .addTemporalMarker(0.0, () -> {
                            //Raise the arm to drop freight at correct height
                            robotBase.lift.arm.setPositionByLevel(shippingHubLevel);
                            robotBase.lift.extender.setPositionByLevel(shippingHubLevel);
                        })
                        .build();

                driveToCarousel = mecanumDrive.trajectoryBuilder(driveToShippingHub.end())
                        .lineToLinearHeading(new Pose2d(-55.5, -60.5, Math.toRadians(0)))

                        .addTemporalMarker(0.5, () -> {
                            //Retract the lift on the way
                            robotBase.lift.extender.setPositionByLevel(0);
                        })
                        .addTemporalMarker(1.5, () -> {
                            //Retract the lift on the way
                            robotBase.lift.claw.close();
                            robotBase.lift.arm.setPositionByLevel(0);
                            robotBase.carouselDrive.run();
                        })
                        .build();

                driveToPark = mecanumDrive.trajectoryBuilder(driveToCarousel.end())
                        //.lineToConstantHeading(new Vector2d(-50.0, -36.0))
                        .lineToLinearHeading(new Pose2d(-54, -38, Math.toRadians(90)))
                        .addTemporalMarker(0, () -> {
                            //Stop any activity on way to park
                            robotBase.carouselDrive.stop();
                        })
                        .build();
                break;

                /***************************************************************************

                 RED RIGHT

                 ***************************************************************************/
            case RED_RIGHT:
                startPose = new Pose2d(12,-63.5,Math.toRadians(90));
                mecanumDrive.setPoseEstimate(startPose);
                driveToShippingHub = mecanumDrive.trajectoryBuilder(startPose)
                        //First add the path segments to follow
                        .splineTo(new Vector2d(-11.0, -42.0), Math.toRadians(90))

                        //Next add the activities during the path (THESE ARE TIME BASED!)
                        .addTemporalMarker(0.0, () -> {
                            //Raise the arm to drop freight at correct height
                            robotBase.lift.arm.setPositionByLevel(shippingHubLevel);
                            robotBase.lift.extender.setPositionByLevel(shippingHubLevel);
                        })
                        .build();

                driveToCarousel = mecanumDrive.trajectoryBuilder(driveToShippingHub.end())
                        //First add the path segments to follow
                        .lineToLinearHeading(new Pose2d(10, -64, Math.toRadians(0)))

                        //Next add the activities during the path (THESE ARE TIME BASED!)
                        .addTemporalMarker(0.5, () -> {
                            robotBase.lift.extender.setPositionByLevel(0);

                        })
                        .addTemporalMarker(1.5, () -> {
                            //Retract the lift on the way
                            robotBase.lift.claw.close();
                            robotBase.lift.arm.setPositionByLevel(0);
                        })

                        .build();

                driveToPark = mecanumDrive.trajectoryBuilder(driveToCarousel.end())
                        .lineToLinearHeading(new Pose2d(36, -64, Math.toRadians(0)))
                        .addTemporalMarker(0, () -> {
                            //robotBase.odometryWheelLift.up();
                        })
                        .build();
                break;

            /***************************************************************************

             DEFAULT

             ***************************************************************************/
            default:
                telemetry.addData("DEFAULT paths for: ", pathName.toString());
                //If we get here, go nowhere to prevent the robot from hurting itself
                startPose = new Pose2d();
                mecanumDrive.setPoseEstimate(startPose);
                driveToShippingHub = mecanumDrive.trajectoryBuilder(startPose)
                        .lineTo(new Vector2d(1, 1))
                        .build();

                driveToCarousel = mecanumDrive.trajectoryBuilder(driveToShippingHub.end())
                        .lineTo(new Vector2d(-1, -1))
                        .build();

                driveToPark = mecanumDrive.trajectoryBuilder(driveToCarousel.end())
                        .lineTo(new Vector2d(1, 1))
                        .build();
                break;

        }

    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose, double maxVelo, double maxAccel) {
        MinVelocityConstraint myVelConstraint = new MinVelocityConstraint(Arrays.asList(new AngularVelocityConstraint(maxAccel),new MecanumVelocityConstraint(maxVelo, 16.66)));
        ProfileAccelerationConstraint myAccelConstraint = new ProfileAccelerationConstraint(maxAccel);
        return new TrajectoryBuilder(startPose, myVelConstraint, myAccelConstraint);
    }
}

