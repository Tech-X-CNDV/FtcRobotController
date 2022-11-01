package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.auton.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.apriltag.AprilTagDetection;

import java.util.ArrayList;

@TeleOp

public class autonomie extends LinearOpMode {
    AprilTagDetectionPipeline parkTag;
    static final double FEET_PER_METER = 3.28084;
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;
    double tagsize = 0.0762;

    int ID_TAG_OF_INTEREST_0 = 0;
    int ID_TAG_OF_INTEREST_1 = 9;
    int ID_TAG_OF_INTEREST_2 = 19;
    AprilTagDetection tagOfInterest = null;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

      /*  Trajectory trajFirstCap = drive.trajectoryBuilder(new Pose2d(35, -60, Math.toRadians(90)))
                .forward(10) //dupa vine scanare Signal
                .splineTo(new Vector2d(28,-7), Math.toRadians(135)) // lasat con
                .back(8)
                .build();
        Trajectory trajConeStack = drive.trajectoryBuilder(trajFirstCap.end().plus(new Pose2d(0, 0, Math.toRadians(-135))))
                .forward(25) //prindere con nou
                .back(12)
                .build();
        Trajectory trajSecondCap = drive.trajectoryBuilder(trajConeStack.end().plus(new Pose2d(0,0,Math.toRadians(-90)))).
                forward(5)//lasare con nou
                .back(5)
                .build();
        Trajectory trajp0 = drive.trajectoryBuilder(trajSecondCap.end()).build();
        Trajectory trajp1 = drive.trajectoryBuilder(trajSecondCap.end())
                .strafeLeft(10)
                .build();
        Trajectory trajp2 = drive.trajectoryBuilder(trajSecondCap.end())
                .strafeRight(10)
                .build();
        Trajectory trajp3 = drive.trajectoryBuilder(trajSecondCap.end())
                .strafeRight(20)
                .build();*/
        int park = 0;

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "camera");
        OpenCvCamera camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);

        camera.setPipeline(parkTag);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(1280,720, OpenCvCameraRotation.UPRIGHT);
            }
            @Override
            public void onError(int errorCode)
            {

            }
        });
        waitForStart();

        ArrayList<AprilTagDetection> currentDetections = parkTag.getLatestDetections();
/*
        drive.followTrajectory(trajFirstCap);
        drive.turn(Math.toRadians(-135));
        drive.followTrajectory(trajConeStack);
        drive.turn(Math.toRadians(-90));
        drive.followTrajectory(trajSecondCap);

        switch (park){
            case 0:
                drive.followTrajectory(trajp0);
                break;
            case 1:
                drive.followTrajectory(trajp1);
                break;
            case 2:
                drive.followTrajectory(trajp2);
                break;
            case 3:
                drive.followTrajectory(trajp3);
                break;
        }*/
        while (opModeIsActive()){

        }
    }
}
