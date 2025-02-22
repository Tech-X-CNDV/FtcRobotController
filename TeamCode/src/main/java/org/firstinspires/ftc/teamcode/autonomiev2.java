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
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.apriltag.AprilTagDetection;

import java.util.ArrayList;

@TeleOp

public class autonomiev2 extends LinearOpMode {
    OpenCvCamera camera;
    WebcamName webcamName;
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
    AprilTagDetection tagOfInterest;
    ArrayList<AprilTagDetection> currentDetections;
    CRobot robot = new CRobot();

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(35, -60, Math.toRadians(90));
        drive.setPoseEstimate(startPose);

        Trajectory trajFirstCap = drive.trajectoryBuilder(startPose)
                .forward(10) //dupa vine scanare Signal
                .splineTo(new Vector2d(27, -4.5), Math.toRadians(135)) // lasat con
                .build();
        Trajectory trajFirstCapReposition = drive.trajectoryBuilder(trajFirstCap.end())
                .back(10).build();
        Trajectory trajConeStack = drive.trajectoryBuilder(trajFirstCapReposition.end().plus(new Pose2d(0, 0, Math.toRadians(-135))))
                .forward(26) //prindere con nou
                .build();
        Trajectory trajConeStackReposition = drive.trajectoryBuilder(trajConeStack.end())
                .back(14).build();
        Trajectory trajSecondCap = drive.trajectoryBuilder(trajConeStackReposition.end().plus(new Pose2d(0, 0, Math.toRadians(-90)))).
                forward(5)//lasare con nou
                .build();
        Trajectory trajSecondCapReposition = drive.trajectoryBuilder(trajSecondCap.end())
                .back(5).build();

        Trajectory trajp0 = drive.trajectoryBuilder(trajSecondCap.end()).forward(1).build();
        Trajectory trajp1 = drive.trajectoryBuilder(trajSecondCap.end())
                .strafeLeft(10)
                .build();
        Trajectory trajp2 = drive.trajectoryBuilder(trajSecondCap.end())
                .strafeRight(10)
                .build();
        Trajectory trajp3 = drive.trajectoryBuilder(trajSecondCap.end())
                .strafeRight(20)
                .build();

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcamName = hardwareMap.get(WebcamName.class, "camera");
        camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
        parkTag = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(parkTag);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

      /* while(!tagFound && !opModeIsActive()) {
       if(currentDetections.size() !=0)
           for(AprilTagDetection tag : currentDetections)
           {
               if(tag.id ==1){tagFound=true;park=1;}
                   else
               if(tag.id == 10){tagFound=true;park=2;}
                   else
               if(tag.id == 19){tagFound=true;park=3;}
                   else park=0;
           }
           telemetry.addData("park detection", park);
       }*/

        telemetry.setMsTransmissionInterval(100);

        int park = 0;
        robot.init(telemetry, hardwareMap);
        //obot.bumperMove(1);
        robot.clawSwitch();

        while (!isStarted() && !isStopRequested()) {
            currentDetections = parkTag.getLatestDetections();
            if (currentDetections.size() != 0) {
                boolean tagFound = false;
                for (AprilTagDetection tag : currentDetections) {
                    if (tag.id == ID_TAG_OF_INTEREST_0 || tag.id == ID_TAG_OF_INTEREST_1 || tag.id == ID_TAG_OF_INTEREST_2) {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }
                if (tagFound) {
                    telemetry.addLine("Tag of interest sighted\n\nLocationData:");
                    tagToTelemetry(tagOfInterest);
                } else {
                    telemetry.addLine("Don't see tag of interest");
                    if (tagOfInterest == null) {
                        telemetry.addLine("(The tag has never been seen)");
                    } else {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }

                switch (tagOfInterest.id) {
                    case 0:
                        park = 1;
                        break;
                    case 9:
                        park = 2;
                        break;
                    case 19:
                        park = 3;
                        break;
                }
            } else {
                telemetry.addLine("Don't see tag of interest :(");

                if (tagOfInterest == null) {
                    telemetry.addLine("(The tag has never been seen)");
                } else {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }
                telemetry.update();
                sleep(10);
            }
        }

        drive.followTrajectory(trajFirstCap);
        drive.followTrajectory(trajFirstCapReposition);
            /*
            drive.turn(Math.toRadians(-135));
            drive.followTrajectory(trajConeStack);
            drive.followTrajectory(trajConeStackReposition);
            drive.turn(Math.toRadians(-90));
            drive.followTrajectory(trajSecondCap);
            drive.followTrajectory(trajSecondCapReposition);
        switch (park) {
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
    }

    void tagToTelemetry(AprilTagDetection detection) {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        //telemetry.addLine(String.format("\nParking Spot = %d", park));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x * FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y * FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z * FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));

    }
}
