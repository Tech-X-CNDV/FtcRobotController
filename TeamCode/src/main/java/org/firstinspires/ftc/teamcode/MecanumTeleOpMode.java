package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;

@TeleOp

public class MecanumTeleOpMode extends OpMode {
    private DcMotor leftFrontMotor = null;
    private DcMotor rightFrontMotor = null;
    private DcMotor leftRearMotor = null;
    private DcMotor rightRearMotor = null;
    private ElapsedTime runtime = new ElapsedTime();
    private double leftStickForward = 0;
    private double leftStickSide = 0;
    private double botSpin = 0;
    private double denominator = 0;
    private double frontLeftPower = 0;
    private double frontRightPower = 0;
    private double rearLeftPower = 0;
    private double rearRightPower = 0;
    int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
    private WebcamName webcam = null;
    //private OpenCvCamera camera = null;

    @Override
    public void init(){
        telemetry.addData("Status", "Initialized");
        leftFrontMotor = hardwareMap.get(DcMotor.class, "leftFrontMotor");
        rightFrontMotor = hardwareMap.get(DcMotor.class, "rightFrontMotor");
        leftRearMotor = hardwareMap.get(DcMotor.class, "leftRearMotor");
        rightRearMotor = hardwareMap.get(DcMotor.class, "rightRearMotor");
        webcam = hardwareMap.get(WebcamName.class, "camera");
       // camera = OpenCvCameraFactory.getInstance().createWebcam(webcam, cameraMonitorViewId);

        telemetry.update();
    }

    @Override
    public void init_loop(){}

    @Override
    public void start(){runtime.reset();}

    @Override
    public void loop(){
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

        leftFrontMotor.setPower(frontLeftPower);
        rightRearMotor.setPower(rearRightPower);

        rightFrontMotor.setPower(frontRightPower);
        leftRearMotor.setPower(rearLeftPower);

        telemetry.update();
    }

    @Override
    public void stop(){}
}

