package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import static java.lang.Thread.sleep;

import java.io.File;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;//
//import org.firstinspires.ftc.teamcode.auton.AprilTagDetectionPipeline;
//import org.openftc.easyopencv.OpenCvCamera;

@Autonomous

public class autonomie extends OpMode {
    private DcMotor leftFrontMotor = null;
    private DcMotor rightFrontMotor = null;
    private DcMotor leftRearMotor = null;
    private DcMotor rightRearMotor = null;

    private DcMotor liftMotor;
    private Servo claw;

    private Servo servoLeft;
    private Servo servoRight;

    File deltaPozFile = AppUtil.getInstance().getSettingsFile("deltaPozFile.txt");
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

        liftMotor = hardwareMap.get(DcMotor.class, "liftMotor");
        claw = hardwareMap.get(Servo.class, "clawServo");
        claw.setPosition(0.6);
        rightFrontMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rightRearMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFrontMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        leftRearMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        liftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        telemetry.update();
        liftMotor.setTargetPosition(0);
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
        runtime.reset();
        leftFrontMotor.setPower(0.75);
        rightFrontMotor.setPower(0.75);
        leftRearMotor.setPower(0.75);
        rightRearMotor.setPower(0.75);
        try {
            sleep(2000);
        } catch (InterruptedException e) {
            e.printStackTrace();
            leftFrontMotor.setPower(0);
            rightFrontMotor.setPower(0);
            leftRearMotor.setPower(0);
            rightRearMotor.setPower(0);

        }
        leftFrontMotor.setPower(0);
        rightFrontMotor.setPower(0);
        leftRearMotor.setPower(0);
        rightRearMotor.setPower(0);
        runtime.reset();
    }

    @Override
    public void loop() {
        telemetry.update();
    }

    @Override
    public void stop() {
    }
}