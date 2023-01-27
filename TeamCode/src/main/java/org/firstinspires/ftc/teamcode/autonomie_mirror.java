package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
//import org.firstinspires.ftc.teamcode.auton.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.apriltag.AprilTagDetection;

import java.util.ArrayList;

@Autonomous

public class autonomie_mirror extends LinearOpMode {
    OpenCvCamera camera;
    WebcamName webcamName;
    AprilTagDetectionPipeline parkTag;
    static final double FEET_PER_METER = 3.28084;
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;
    double tagsize = 0.045;

    int ID_TAG_OF_INTEREST_0 = 0;
    int ID_TAG_OF_INTEREST_1 = 9;
    int ID_TAG_OF_INTEREST_2 = 19;
    AprilTagDetection tagOfInterest;

    int park = 0;

    CRobot robot = new CRobot();

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(-35, -60, Math.toRadians(90));
        drive.setPoseEstimate(startPose);
        Trajectory trajSignalDisplacement = drive.trajectoryBuilder(startPose)
                .forward(55)
                .build();
        Trajectory trajSignalReposition = drive.trajectoryBuilder(trajSignalDisplacement.end())
                .addDisplacementMarker(() -> {
                    robot.bumperMove(1);
                    robot.runLift(14);
                })
                .back(10)
                .build();
        Trajectory trajFirstCap = drive.trajectoryBuilder(trajSignalReposition.end())
                .splineTo(new Vector2d(-26, -3), Math.toRadians(45))// lasat con
                //.forward(10)
                .build();
        Trajectory trajFirstCapReposition = drive.trajectoryBuilder(trajFirstCap.end())
                .back(12)
                .build();
        Trajectory trajConeStack = drive.trajectoryBuilder(trajFirstCapReposition.end().plus(new Pose2d(0, 0, Math.toRadians(135))))
                .forward(26)
                .build();
        Trajectory trajConeStackReposition = drive.trajectoryBuilder(trajConeStack.end())
                .back(26)
                .build();
        Trajectory trajSecondCap = drive.trajectoryBuilder(trajConeStackReposition.end().plus(new Pose2d(0, 0, Math.toRadians(-135)))).
                forward(12)
                .build();
        Trajectory trajSecondCapReposition = drive.trajectoryBuilder(trajSecondCap.end())
                .back(5).build();

        Trajectory trajp0 = drive.trajectoryBuilder(trajFirstCapReposition.end().plus(new Pose2d(0, 0, Math.toRadians(45)))).forward(1).build();
        Trajectory trajp1 = drive.trajectoryBuilder(trajFirstCapReposition.end().plus(new Pose2d(0, 0, Math.toRadians(45))))
                .strafeLeft(24)
                .build();
        Trajectory trajp2 = drive.trajectoryBuilder(trajFirstCapReposition.end().plus(new Pose2d(0, 0, Math.toRadians(45))))
                .back(1)
                .build();
        Trajectory trajp3 = drive.trajectoryBuilder(trajFirstCapReposition.end().plus(new Pose2d(0, 0, Math.toRadians(45))))
                .strafeRight(24)
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


        while (!isStarted() && !isStopRequested()) {
            ArrayList<AprilTagDetection> currentDetections = parkTag.getLatestDetections();
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
                sleep(10);
            }
            telemetry.update();

        }

        robot.init(telemetry, hardwareMap);
        robot.liftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.bumperMove(1);
        robot.clawSwitch();
        drive.followTrajectory(trajSignalDisplacement);
        drive.followTrajectory(trajSignalReposition);
        drive.followTrajectory(trajFirstCap);
        while(robot.liftMotor.getCurrentPosition()!=robot.liftMotor.getTargetPosition()){}
        robot.clawSwitch();
        sleep(1000);
        drive.followTrajectory(trajFirstCapReposition);
        robot.runLift(9);
        drive.turn(Math.toRadians(135));
        drive.followTrajectory(trajConeStack);
        drive.followTrajectory(trajConeStackReposition);
        drive.turn(Math.toRadians(-135));
        drive.followTrajectory(trajSecondCap);
        /*
        drive.turn(Math.toRadians(-90));
        drive.followTrajectory(trajSecondCapReposition);*/

        switch (park) {
            case 0:
                drive.followTrajectory(trajp0);
                while(robot.liftMotor.getCurrentPosition()!=robot.liftMotor.getTargetPosition()){}
                break;
            case 1:
                drive.followTrajectory(trajp1);
                while(robot.liftMotor.getCurrentPosition()!=robot.liftMotor.getTargetPosition()){}
                break;
            case 2:
                drive.followTrajectory(trajp2);
                while(robot.liftMotor.getCurrentPosition()!=robot.liftMotor.getTargetPosition()){}
                break;
            case 3:
                drive.followTrajectory(trajp3);
                while(robot.liftMotor.getCurrentPosition()!=robot.liftMotor.getTargetPosition()){}
                break;
        }

    }

    void tagToTelemetry(AprilTagDetection detection) {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("\nParking Spot = %d", park));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x * FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y * FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z * FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));

    }
}
/*
Salut ! Mă numesc Luca si fac parte din departamentul de programare. La fel ca si corpul robotului, creierul acestuia, adica programele folosite in insufletirea acestui s-au schimbat si adaptat la fel de mult.

Pentru cod, am folosit aplicatia Android Studio, in timp ce in anii trecuti foloseam OnBotJava. Aceasta alegere a fost motivata de diversele imbunatatiri pe care le aduce Android Studio, precum integrarea GitHub ului ce ne a ajutat enorm in fluidizarea procesului de coding.

Pentru TeleOp, am creat un cod in care am segmentat fiecare actiune a robotului in cate o functie. Acest lucru ne a permis sa ne focusam pe asigurarea si imbunatatirea controlului driverilor fara a ne complica prea mult cu structurarea. Pe parcursul realizarii codului, am comunicat cu driverii, unul dintre ei fiind eu, pentru a face dirijarea robotului o munca cat mai usoara si intuitiva.

Pentru autonomie, folosind modulele de odometrie si libraria roadrunner am dat viata robotului in acele 30 de secunde de aur. Procesul realizarii codului pentru autonomie a fost unul foarte amanuntit, bazat pe mai multe etape. In primul rand, am gandit o strategie de joc a robotului, bazata pe capabilitatiile robotului. Dupa aceea, am trecut strategia respectiva prin libraria MeepMeep, cu ajutorul careia am putut analiza o simulare aproximativa a  robotului pe teren. In final, am aplicat si am modificat codul final pe robotul fizic.

*/