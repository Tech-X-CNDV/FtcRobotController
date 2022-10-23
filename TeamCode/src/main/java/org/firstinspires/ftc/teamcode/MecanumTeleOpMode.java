package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.auton.AprilTagDetectionPipeline;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.firstinspires.ftc.teamcode.auton.AprilTagDetectionPipeline;


@TeleOp

public class MecanumTeleOpMode extends OpMode {
    private DcMotor leftFrontMotor = null;
    private DcMotor rightFrontMotor = null;
    private DcMotor leftRearMotor = null;
    private DcMotor rightRearMotor = null;

    private DcMotor liftMotor;
    private Servo claw;

    private ElapsedTime runtime = new ElapsedTime();
    private double leftStickForward = 0;
    private double leftStickSide = 0;
    private double botSpin = 0;
    private double denominator = 0;
    private double frontLeftPower = 0;
    private double frontRightPower = 0;
    private double rearLeftPower = 0;
    private double rearRightPower = 0;
   // int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
    private WebcamName webcam = null;
    private OpenCvCamera camera = null;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;
    double tagsize = 0.166;

    @Override
    public void init(){
        telemetry.addData("Status", "Initialized");
        leftFrontMotor = hardwareMap.get(DcMotor.class, "leftFrontMotor");
        rightFrontMotor = hardwareMap.get(DcMotor.class, "rightFrontMotor");
        leftRearMotor = hardwareMap.get(DcMotor.class, "leftRearMotor");
        rightRearMotor = hardwareMap.get(DcMotor.class, "rightRearMotor");

        liftMotor = hardwareMap.get(DcMotor.class, "liftMotor");
        claw = hardwareMap.get(Servo.class, "clawServo");
        claw.setPosition(0);
        liftMotor.setTargetPosition(400);

        webcam = hardwareMap.get(WebcamName.class, "camera");
        // camera = OpenCvCameraFactory.getInstance().createWebcam(webcam, cameraMonitorViewId);

        rightFrontMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        rightRearMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        leftFrontMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRearMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.update();
    }

    @Override
    public void init_loop(){}

    @Override
    public void start(){runtime.reset();}

    @Override
    public void loop(){
        int targetPosition = 300;
        boolean clawExtended = false;

        leftStickForward = -this.gamepad1.left_stick_y;
        leftStickSide = this.gamepad1.left_stick_x * 1.1;
        botSpin = this.gamepad1.right_stick_x;

        denominator = Math.max(Math.abs(leftStickForward) + Math.abs(leftStickSide) + Math.abs(botSpin), 1);
        frontLeftPower = (leftStickForward + leftStickSide + botSpin) / denominator;
        rearRightPower = (leftStickForward + leftStickSide - botSpin) / denominator;
        frontRightPower = (leftStickForward - leftStickSide - botSpin) / denominator;
        rearLeftPower = (leftStickForward - leftStickSide + botSpin) / denominator;

        telemetry.addData("Left Stick Y", leftStickForward);
        telemetry.addData("Left Stick X", leftStickSide);

        if(this.gamepad1.a){
            liftMotor.setDirection(DcMotorSimple.Direction.FORWARD);

            liftMotor.setPower(1);
        }
        if(this.gamepad1.b){
            liftMotor.setPower(0);
        }
        if(this.gamepad1.y){
            liftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            liftMotor.setPower(1);
        }
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        telemetry.addData("Lift encoder", liftMotor.getCurrentPosition());
        if (liftMotor.getCurrentPosition() == targetPosition)
            telemetry.addLine("EXTENDED");

        //prindere gheara
        if (this.gamepad1.x){
            if(!clawExtended){
                telemetry.addLine("Claw EXTENDED");
                claw.setPosition(.5);
                clawExtended = true;
            } else {
                telemetry.addLine("Claw RETRACTED");
                claw.setPosition(0);
                clawExtended = false;
            }
        }
        telemetry.addData("Servo Position", claw.getPosition());

        leftFrontMotor.setPower(frontLeftPower);
        rightRearMotor.setPower(rearRightPower);

        rightFrontMotor.setPower(frontRightPower);
        leftRearMotor.setPower(rearLeftPower);

        telemetry.update();
    }

    @Override
    public void stop(){}
}

