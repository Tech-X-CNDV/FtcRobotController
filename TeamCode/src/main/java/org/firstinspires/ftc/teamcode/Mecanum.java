package org.firstinspires.ftc.teamcode;

//import com.arcrobotics.ftclib.drivebase.MecanumDrive;
//import com.arcrobotics.ftclib.gamepad.GamepadEx;
//
import com.acmerobotics.roadrunner.drive.MecanumDrive;
//import com.arcrobotics.ftclib.gamepad.GamepadEx;
//import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import java.io.File;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;//
//import org.firstinspires.ftc.teamcode.auton.AprilTagDetectionPipeline;
//import org.openftc.easyopencv.OpenCvCamera;

@TeleOp

public class Mecanum extends OpMode {

    private DcMotor leftFrontMotor = null; //
    private DcMotor rightFrontMotor = null;
    private DcMotor leftRearMotor = null;
    private DcMotor rightRearMotor = null;

    //private MecanumDrive drive;
    //private GamepadEx driverOp;

    private DcMotor liftMotor;
    private Servo claw;
    private Servo servoLeft;
    private Servo servoRight;

   // File deltaPozFile = AppUtil.getInstance().getSettingsFile("deltaPozFile.txt");
    private ElapsedTime runtime = new ElapsedTime();
    private double leftStickForward = 0;
    private double leftStickSide = 0;
    private double botSpin = 0;
    private double denominator = 0;
    private double frontLeftPower = 0;
    private double frontRightPower = 0;
    private double rearLeftPower = 0;
    private double rearRightPower = 0;
    boolean clawExtended = false;
    boolean moveX = false, moveA = false, moveY = false, moveLbumper = false, bumperExtended = false, moveRbumper = false;
    int position[] = {500, 700, 1800, 3200, 4800};
    int vPos = 0;
    int targetPosition;
    @Override
    public void init() {

        telemetry.addData("Status", "Initialized");
        leftFrontMotor = hardwareMap.get(DcMotor.class, "leftFrontMotor");
        rightFrontMotor = hardwareMap.get(DcMotor.class, "rightFrontMotor");
        leftRearMotor = hardwareMap.get(DcMotor.class, "leftRearMotor");
        rightRearMotor = hardwareMap.get(DcMotor.class, "rightRearMotor");

        servoLeft = hardwareMap.get(Servo.class, "servoLeft");
        servoRight = hardwareMap.get(Servo.class, "servoRight");
        servoLeft.setPosition(0);
        servoRight.setPosition(1);
        liftMotor = hardwareMap.get(DcMotor.class,  "liftMotor");
        claw = hardwareMap.get(Servo.class, "claw");
        claw.setPosition(0);
        leftFrontMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        leftRearMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        rightFrontMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rightRearMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        liftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        telemetry.update();
        liftMotor.setTargetPosition(0);
        // ???? beta testing
        //drive = new MecanumDrive(true,leftFrontMotor, rightFrontMotor, leftRearMotor, rightRearMotor);
        //driverOp = new GamepadEx(gamepad2);
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
        runtime.reset();
    }

    @Override
    public void loop() {
        leftStickForward = leftStickForward - this.gamepad2.left_stick_y/0.01;
        if(this.gamepad2.left_stick_y==0)leftStickForward=0;
        leftStickSide =this.gamepad2.left_stick_x;
        botSpin =this.gamepad2.right_stick_x;
        denominator = Math.max(Math.abs(leftStickForward) + Math.abs(leftStickSide) + Math.abs(botSpin), 1);
        frontLeftPower = (leftStickForward + leftStickSide + botSpin) / denominator;
        rearRightPower = (leftStickForward + leftStickSide - botSpin) / denominator;
        frontRightPower = (leftStickForward - leftStickSide - botSpin) / denominator;
        rearLeftPower = (leftStickForward - leftStickSide + botSpin) / denominator;

        telemetry.addData("Left Stick Y", leftStickForward);
        telemetry.addData("Left Stick X", leftStickSide);
        telemetry.addData("Lift target", liftMotor.getTargetPosition());
        if (this.gamepad1.back) {
            liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotor.setPower(1);

        if (this.gamepad1.right_trigger > 0) liftMotor.setTargetPosition(-10000);
        else if (this.gamepad1.left_trigger > 0) liftMotor.setTargetPosition(10000);
        else liftMotor.setTargetPosition(liftMotor.getCurrentPosition());

        telemetry.addData("Lift encoder", liftMotor.getCurrentPosition());
        //bumpere
        if(this.gamepad1.right_bumper && moveRbumper)
        {
         servoLeft.setPosition(0);
         servoRight.setPosition(1);
         moveRbumper = false;
        }
        if(!this.gamepad1.right_bumper)moveRbumper=true;
        if(this.gamepad1.left_bumper && moveLbumper)
        {
            servoLeft.setPosition(1);
            servoRight.setPosition(0);
            moveLbumper = false;
        }
        if(!this.gamepad1.left_bumper)moveLbumper=true;
        //prindere gheara
        if (this.gamepad1.x && moveX == true) {
            if (!clawExtended) {
                telemetry.addLine("Claw EXTENDED");
                claw.setPosition(0);
                clawExtended = true;
            } else {
                telemetry.addLine("Claw RETRACTED");
                claw.setPosition(0.6);
                clawExtended = false;
            }
            moveX = false;
        }
        if (!this.gamepad1.x) moveX = true;
        telemetry.addData("Servo Position", claw.getPosition());
        leftFrontMotor.setPower(frontLeftPower);
        rightRearMotor.setPower(rearRightPower);
        rightFrontMotor.setPower(frontRightPower);
        leftRearMotor.setPower(rearLeftPower);
        telemetry.update();
    }

    @Override
    public void stop() {
    }
}