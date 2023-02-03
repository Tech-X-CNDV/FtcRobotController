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

import org.apache.commons.math3.geometry.euclidean.twod.Vector2D;
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
    int stack = 1;
    CRobot robot = new CRobot();

    public void Cap() {
        //while (robot.liftMotor.getCurrentPosition() <= robot.liftMotor.getTargetPosition() - 2) {}
        robot.clawSwitch();
        sleep(300);
    }

    public void Stack() {
        robot.runLiftStack(stack);
        stack++;
        while (robot.liftMotor.getCurrentPosition() >= robot.liftMotor.getTargetPosition()) {
        }
        robot.clawSwitch();
        sleep(400);
        robot.runLiftStack(stack - 2);
        while (robot.liftMotor.getCurrentPosition() <= robot.liftMotor.getTargetPosition() - 5) {
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {
        /*
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(-35, -60, Math.toRadians(90));
        drive.setPoseEstimate(startPose);
        Trajectory trajSignalDisplacement = drive.trajectoryBuilder(startPose)
                .addDisplacementMarker(() -> {
                    robot.runLift(11);
                })
                .forward(55)
                .build();
        Trajectory trajSignalReposition = drive.trajectoryBuilder(trajSignalDisplacement.end())
                .back(10)
                .build();
        Trajectory trajFirstCap = drive.trajectoryBuilder(trajSignalReposition.end())
                .splineTo(new Vector2d(-28, -2.5), Math.toRadians(45))// lasat con
                //.forward(10)
                .build();
        //Variant 2
        Trajectory FirstCapRepoV3 = drive.trajectoryBuilder(trajFirstCap.end())
                .lineToSplineHeading(new Pose2d(-35,-10,Math.toRadians(180)))
                .addDisplacementMarker(() -> {
                    sleep(100);
                    robot.runLiftStack(0);
                })
                //.lineToSplineHeading(new Pose2d(-35,-15,Math.toRadians(90)))
                .build();
        Trajectory ConeStackV3 = drive.trajectoryBuilder(FirstCapRepoV3.end())
                //.splineTo(new Vector2d(-35,-9.5),Math.toRadians(180))
                .forward(30)
                .build();
        Trajectory ConeStackRetractV3 = drive.trajectoryBuilder(ConeStackV3.end())
                .addDisplacementMarker(() -> {
            robot.runLift(11);
        })
                .back(30)
                .build();
        Trajectory ConeStackRepoV3= drive.trajectoryBuilder(ConeStackRetractV3.end())
                .lineToSplineHeading(new Pose2d(-35,-15,Math.toRadians(90)))
                .build();
        Trajectory SecondCapV3 = drive.trajectoryBuilder(ConeStackRetractV3.end())
                .splineTo(new Vector2d(-29, -3.5), Math.toRadians(45))// lasat con
                .build();
        Trajectory SecondCapRepoV3 = drive.trajectoryBuilder(SecondCapV3.end())
                .lineToSplineHeading(new Pose2d(-35,-9.5,Math.toRadians(180)))
                .addDisplacementMarker(() -> {
                    sleep(100);
                    robot.runLiftStack(0);
                })
                //.lineToSplineHeading(new Pose2d(-35,-15,Math.toRadians(90)))
                .build();
        Trajectory ConeStackSecondV3 = drive.trajectoryBuilder(SecondCapRepoV3.end())
                //.splineTo(new Vector2d(-35,-9.5),Math.toRadians(180))
                .forward(30)
                .build();
        Trajectory ConeStackSecondRetractV3 = drive.trajectoryBuilder(ConeStackSecondV3.end())
                .addDisplacementMarker(() -> {
                    robot.runLift(11);
                })
                .back(30)
                .build();
        Trajectory ThirdCapV3 = drive.trajectoryBuilder(ConeStackRetractV3.end())
                .splineTo(new Vector2d(-29, -3.5), Math.toRadians(45))// lasat con
                .build();
        //
        Trajectory trajFirstCapReposition = drive.trajectoryBuilder(trajFirstCap.end())
                .back(14)
                .build();
        Trajectory trajConeStack = drive.trajectoryBuilder(trajFirstCapReposition.end().plus(new Pose2d(0, 0, Math.toRadians(135))))
                .forward(27)
                .build();
        //Variant
        Trajectory trajConeStackRepositionV2 = drive.trajectoryBuilder(ConeStackV3.end())
                .addDisplacementMarker(() -> {
                    robot.runLift(11);
                })
                .back(50)
                .build();
        Trajectory trajSecondCapV2 = drive.trajectoryBuilder(trajConeStackRepositionV2.end())
                .splineTo(new Vector2d(-22, -5), Math.toRadians(135))
                .build();
        Trajectory trajSecondCapRepositionV2 = drive.trajectoryBuilder(trajSecondCapV2.end())
                .back(16).build();
        Trajectory trajConeStack2V2 = drive.trajectoryBuilder(trajSecondCapRepositionV2.end())
                .splineTo(new Vector2d(-27,trajConeStack.end().getY()),Math.toRadians(180))
                .addDisplacementMarker(() -> {
                    robot.runLiftStack(0);
                })
                .forward(-trajConeStack.end().getX()-27)
                .build();
        Trajectory trajConeStackReposition2V2 = drive.trajectoryBuilder(trajConeStack2V2.end())
                .addDisplacementMarker(() -> {
                    robot.runLift(11);
                })
                .back(50)
                .build();
        Trajectory trajThirdCapV2 = drive.trajectoryBuilder(trajConeStackReposition2V2.end())
                .splineTo(new Vector2d(-21, -5), Math.toRadians(135))
                .build();
        //
        Trajectory trajConeStackReposition = drive.trajectoryBuilder(trajConeStack.end())
                .back(23)
                .addDisplacementMarker(() -> {
                    robot.runLift(11);
                })
                .build();
        Trajectory trajSecondCap = drive.trajectoryBuilder(trajConeStackReposition.end().plus(new Pose2d(0, 0, Math.toRadians(-135)))).
                forward(13)
                .build();
        Trajectory trajSecondCapReposition = drive.trajectoryBuilder(trajSecondCap.end())
                .back(13).build();
        Trajectory trajConeStack2 = drive.trajectoryBuilder(trajSecondCapReposition.end().plus(new Pose2d(0, 0, Math.toRadians(135))))
                .forward(23)
                .build();

        Trajectory trajp0 = drive.trajectoryBuilder(trajSecondCapReposition.end())
                .back(1)
                .build();
        Trajectory trajp1 = drive.trajectoryBuilder(trajSecondCapRepositionV2.end())
                .splineTo(new Vector2d(-27,trajConeStack.end().getY()),Math.toRadians(135))
                .build();
        Trajectory trajp2 = drive.trajectoryBuilder(trajSecondCapRepositionV2.end())
                .splineTo(new Vector2d(-13,trajConeStack.end().getY()),Math.toRadians(135))
                .build();
        Trajectory trajp3 = drive.trajectoryBuilder(trajSecondCapRepositionV2.end())
                .back(1)
                .build();
                */
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

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(-35, -60, Math.toRadians(90));
        drive.setPoseEstimate(startPose);
        Trajectory SignalDisplacement = drive.trajectoryBuilder(startPose)
                .addDisplacementMarker(() -> {
                    robot.runLift(11);
                })
                .forward(55)
                .build();
        Trajectory SignalReposition = drive.trajectoryBuilder(SignalDisplacement.end())
                .back(6)
                .build();
        Trajectory FirstCap = drive.trajectoryBuilder(SignalReposition.end())
                //.splineTo(new Vector2d(-27.5, -3.5), Math.toRadians(45))// lasat con
                .splineTo(new Vector2d(-31.5, -2.5), Math.toRadians(45))
                .build();
        Trajectory FirstCapRepo = drive.trajectoryBuilder(FirstCap.end())
                .addDisplacementMarker(5, () -> {
                    robot.runLiftStack(0);
                })
                .lineToSplineHeading(new Pose2d(-40, -11, Math.toRadians(180)))
                .build();
        Trajectory FirstStack = drive.trajectoryBuilder(FirstCapRepo.end())
                .forward(26)
                .build();
        Trajectory FirstStackRepo = drive.trajectoryBuilder(FirstStack.end())
                .addDisplacementMarker(() -> {
                    robot.runLift(11);
                })
                .back(30)
                .build();
        Trajectory SecondCap = drive.trajectoryBuilder(FirstStackRepo.end())
                // .lineToSplineHeading(new Pose2d(-29, -5, Math.toRadians(45)))
                .lineToSplineHeading(new Pose2d(-33.5, -3, Math.toRadians(45)))
                .build();
        Trajectory SecondCapRepo = drive.trajectoryBuilder(SecondCap.end())
                .addDisplacementMarker(5, () -> {
                    // robot.rotateClaw();
                    robot.runLiftStack(0);
                })
                .lineToSplineHeading(new Pose2d(-40, -11, Math.toRadians(180)))
                .build();
        Trajectory SecondStack = drive.trajectoryBuilder(SecondCapRepo.end())
                .forward(26)
                .build();
        Trajectory SecondStackRepo = drive.trajectoryBuilder(SecondStack.end())
                .addDisplacementMarker(() -> {
                    robot.runLift(11);
                })
                .back(30)
                .build();
        Trajectory ThirdCap = drive.trajectoryBuilder(SecondStackRepo.end())
                .addDisplacementMarker(() -> {
                    // robot.rotateClaw();
                })
                .lineToSplineHeading(new Pose2d(-33.5, -3, Math.toRadians(45)))
                // .lineToSplineHeading(new Pose2d(-29, -5, Math.toRadians(45)))

                .build();
        Trajectory ThirdCapRepo = drive.trajectoryBuilder(ThirdCap.end())
                .addDisplacementMarker(5, () -> {
                    //  robot.rotateClaw();
                    robot.runLiftStack(0);
                })
                .lineToSplineHeading(new Pose2d(-40, -11, Math.toRadians(180)))
                .build();
        Trajectory ThirdStack = drive.trajectoryBuilder(ThirdCapRepo.end())
                .forward(30)
                .build();
        Trajectory ThirdStackRepo = drive.trajectoryBuilder(ThirdStack.end())
                .addDisplacementMarker(() -> {
                    robot.runLift(11);
                })
                .back(30)
                .build();
        Trajectory FourthCap = drive.trajectoryBuilder(ThirdStackRepo.end())
                .addDisplacementMarker(() -> {
                    robot.rotateClaw();
                })
                .lineToSplineHeading(new Pose2d(-31.5, -5.5, Math.toRadians(225)))
                .build();
        Trajectory trajp2 = drive.trajectoryBuilder(ThirdCap.end())
                .addDisplacementMarker(3, () -> {
                    //  robot.rotateClaw();
                    robot.runLiftStack(0);
                })
                .back(10)
                .build();
        Trajectory trajp2ex = drive.trajectoryBuilder(trajp2.end())
                .lineToSplineHeading(new Pose2d(-40, -11, Math.toRadians(180)))
                .build();
        Trajectory trajp1 = drive.trajectoryBuilder(ThirdCap.end())
                .addDisplacementMarker(5, () -> {
                    //  robot.rotateClaw();
                    robot.runLiftStack(0);
                })
                .lineToSplineHeading(new Pose2d(-40, -11, Math.toRadians(180)))
                .build();
        Trajectory trajp1ex = drive.trajectoryBuilder(trajp1.end())
                .forward(25)
                .build();
        Trajectory trajp3 = drive.trajectoryBuilder(ThirdCap.end())
                .addDisplacementMarker(3, () -> {
                    //  robot.rotateClaw();
                    robot.runLiftStack(0);
                })
                .back(10)
                .build();
        Trajectory trajp3ex = drive.trajectoryBuilder(trajp3.end())
                .lineToSplineHeading(new Pose2d(-14, -11, Math.toRadians(-90)))
                .build();
        robot.init(telemetry, hardwareMap);
        robot.liftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.bumperMove(1);
        robot.servoRotator.setPosition(0);
        robot.clawSwitch();
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
        drive.followTrajectory(SignalDisplacement);
        drive.followTrajectory(SignalReposition);
        drive.followTrajectory(FirstCap);
        Cap();
        drive.followTrajectory(FirstCapRepo);
        drive.followTrajectory(FirstStack);
        Stack();
        drive.followTrajectory(FirstStackRepo);
        drive.followTrajectory(SecondCap);
        Cap();
        drive.followTrajectory(SecondCapRepo);
        drive.followTrajectory(SecondStack);
        Stack();
        drive.followTrajectory(SecondStackRepo);
        drive.followTrajectory(ThirdCap);
        Cap();
        switch (park) {
            case 0:
                drive.followTrajectory(trajp2);
                drive.followTrajectory(trajp2ex);
                while (robot.liftMotor.getCurrentPosition() != robot.liftMotor.getTargetPosition()) {
                }
                break;
            case 1:
                drive.followTrajectory(trajp1);
                drive.followTrajectory(trajp1ex);
                while (robot.liftMotor.getCurrentPosition() != robot.liftMotor.getTargetPosition()) {
                }
                break;
            case 2:
                drive.followTrajectory(trajp2);
                drive.followTrajectory(trajp2ex);
                while (robot.liftMotor.getCurrentPosition() != robot.liftMotor.getTargetPosition()) {
                }
                break;
            case 3:
                drive.followTrajectory(trajp3);
                drive.followTrajectory(trajp3ex);
                while (robot.liftMotor.getCurrentPosition() != robot.liftMotor.getTargetPosition()) {
                }
                break;
        }
        //1150 , 850 590 330 90      0,
        //
        /*
        drive.followTrajectory(trajSignalDisplacement);
        drive.followTrajectory(trajSignalReposition);
        //
        drive.followTrajectory(trajFirstCap);
       // while(robot.liftMotor.getCurrentPosition()<=robot.liftMotor.getTargetPosition()){}
        robot.clawSwitch();
        sleep(200);
        drive.followTrajectory(FirstCapRepoV3);
        drive.followTrajectory(ConeStackV3);
        robot.runLiftStack(stack);stack++;
        while(robot.liftMotor.getCurrentPosition()>=robot.liftMotor.getTargetPosition()){}
        robot.clawSwitch();
        sleep(200);
        robot.runLiftStack(stack-2);
        while(robot.liftMotor.getCurrentPosition()<=robot.liftMotor.getTargetPosition()){}
        drive.followTrajectory(trajConeStackRepositionV2);
        drive.followTrajectory(trajSecondCapV2);
        while(robot.liftMotor.getCurrentPosition()<=robot.liftMotor.getTargetPosition()-5){}
        robot.clawSwitch();
        sleep(200);
        drive.followTrajectory(trajSecondCapRepositionV2);
        drive.followTrajectory(trajConeStack2V2);
        robot.runLiftStack(stack);stack++;
        while(robot.liftMotor.getCurrentPosition()>=robot.liftMotor.getTargetPosition()+10){}
        robot.clawSwitch();
        sleep(200);
        robot.runLiftStack(stack-2);
        drive.followTrajectory(trajConeStackReposition2V2);
        drive.followTrajectory(trajThirdCapV2);
        while(robot.liftMotor.getCurrentPosition()<=robot.liftMotor.getTargetPosition()-5){}
        robot.clawSwitch();
        sleep(200);

        drive.followTrajectory(trajFirstCapReposition);
        */
        /*
        drive.followTrajectory(FirstCapRepoV3);
        drive.followTrajectory(ConeStackV3);
        robot.runLiftStack(stack);stack++;
        while(robot.liftMotor.getCurrentPosition()>=robot.liftMotor.getTargetPosition()){}
        robot.clawSwitch();
        sleep(300);
        robot.runLiftStack(stack - 2);
        while(robot.liftMotor.getCurrentPosition()<=(robot.liftMotor.getTargetPosition()-10)){}
        drive.followTrajectory(ConeStackRetractV3);
       // drive.followTrajectory(FirstCapRepoV3);
        drive.followTrajectory(SecondCapV3);
        robot.clawSwitch();
        sleep(300);
        drive.followTrajectory(SecondCapRepoV3);
        drive.followTrajectory(ConeStackSecondV3);
        robot.runLiftStack(stack);stack++;
        while(robot.liftMotor.getCurrentPosition()>=robot.liftMotor.getTargetPosition()){}
        robot.clawSwitch();
        sleep(300);
        robot.runLiftStack(stack - 2);
        while(robot.liftMotor.getCurrentPosition()<=(robot.liftMotor.getTargetPosition()-10)){}
        drive.followTrajectory(ConeStackSecondRetractV3);
        while(robot.liftMotor.getCurrentPosition()<=robot.liftMotor.getTargetPosition()){}
        drive.followTrajectory(ThirdCapV3);
        sleep(500);
        robot.clawSwitch();
        sleep(200);
        */
        /*
        drive.turn(Math.toRadians(135));
        drive.followTrajectory(trajConeStack);
        robot.runLiftStack(stack);stack++;
        while(robot.liftMotor.getCurrentPosition()>=robot.liftMotor.getTargetPosition()){}
        robot.clawSwitch();
        sleep(300);
        robot.runLiftStack(stack - 2);
        while(robot.liftMotor.getCurrentPosition()<=(robot.liftMotor.getTargetPosition()-10)){}
        drive.followTrajectory(trajConeStackRepositionV2);
        drive.followTrajectory(trajSecondCapV2);
        robot.clawSwitch();
        sleep(300);
        drive.followTrajectory(trajSecondCapRepositionV2);
        drive.followTrajectory(trajConeStack2V2);
        robot.runLiftStack(stack);stack++;
        while(robot.liftMotor.getCurrentPosition()>=robot.liftMotor.getTargetPosition()){}
        robot.clawSwitch();
        sleep(300);
        robot.runLiftStack(stack - 2);
        while(robot.liftMotor.getCurrentPosition()<=(robot.liftMotor.getTargetPosition()-10)){}
        drive.followTrajectory(trajConeStackReposition2V2);
        while(robot.liftMotor.getCurrentPosition()<=robot.liftMotor.getTargetPosition()){}
        drive.followTrajectory(trajThirdCapV2);
        sleep(500);
        robot.clawSwitch();
        sleep(200);
        */
        /*
        drive.followTrajectory(trajSecondCapRepositionV2);
        robot.runLiftStack(stack - 3);
        while(robot.liftMotor.getCurrentPosition()<=(robot.liftMotor.getTargetPosition()-10)){}
        */
        /*
        drive.turn(Math.toRadians(-90));
        drive.followTrajectory(trajSecondCapReposition);*/
        /*
        switch (park) {
            case 0:
                drive.followTrajectory(trajp0);
                while (robot.liftMotor.getCurrentPosition() != robot.liftMotor.getTargetPosition()) {
                }
                break;
            case 1:
                drive.followTrajectory(trajp1);
                while (robot.liftMotor.getCurrentPosition() != robot.liftMotor.getTargetPosition()) {
                }
                break;
            case 2:
                drive.followTrajectory(trajp2);
                while (robot.liftMotor.getCurrentPosition() != robot.liftMotor.getTargetPosition()) {
                }
                break;
            case 3:
                drive.followTrajectory(trajp3);
                while (robot.liftMotor.getCurrentPosition() != robot.liftMotor.getTargetPosition()) {
                }
                break;
        }
        */
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