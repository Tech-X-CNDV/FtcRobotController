//package org.firstinspires.ftc.teamcode;
/*
import android.os.DropBoxManager;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.camera.delegating.DelegatingCaptureSequence;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.CRobot;
import org.firstinspires.ftc.teamcode.auton.AprilTagDetectionPipeline;
import org.openftc.easyopencv.OpenCvCamera;
@Disabled
class CRobotp {
    public DcMotor leftFrontMotor;
    public DcMotor rightFrontMotor;
    public DcMotor leftRearMotor;
    public DcMotor rightRearMotor;
    public DcMotor liftMotor;
    public Servo claw;
    public Servo servoLeft;
    public Servo servoRight;

    public void init(Telemetry telemetry, HardwareMap hardwareMap) {
        telemetry.addData("Status", "Initialized");
        leftFrontMotor = hardwareMap.get(DcMotor.class, "leftFrontMotor");
        rightFrontMotor = hardwareMap.get(DcMotor.class, "rightFrontMotor");
        leftRearMotor = hardwareMap.get(DcMotor.class, "leftRearMotor");
        rightRearMotor = hardwareMap.get(DcMotor.class, "rightRearMotor");
        liftMotor = hardwareMap.get(DcMotor.class, "liftMotor");
        claw = hardwareMap.get(Servo.class, "clawServo");
        claw.setPosition(0.6);
        liftPos = 8;
        servoLeft = hardwareMap.get(Servo.class, "servoLeft");
        servoRight = hardwareMap.get(Servo.class, "servoRight");
        servoLeft.setPosition(0.3);
        servoRight.setPosition(0.7);
        rightFrontMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        rightRearMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        leftFrontMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRearMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        liftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        telemetry.update();
    }

    boolean clawExtended = false;
    boolean liftPower = true;
    boolean overrideLift = true;
    int position[] = {-7420, -5700, -4600, -3300, -2500, -1100, -600, 0, 600, 1100, 2500, 3300, 4600, 5700, 7430};
    public int liftPos = 8, bumperPos = 0;

    public void targetLiftUp() {
        if (liftPos < 14) {
            liftPos++;
        }
    }

    public void targetLiftDown() {
        if (liftPos > 0) {
            liftPos--;
        }
    }

    public void powerLift() {
        liftPower = !liftPower;
    }

    public void runLift(int liftPos) {
        liftMotor.setTargetPosition(position[liftPos]);
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        if (liftPower) liftMotor.setPower(1);
        else liftMotor.setPower(0);
    }

    public void overrideLiftUp() {
        liftMotor.setPower(1);
        liftMotor.setTargetPosition(10000);
    }

    public void overrideLiftDown() {
        liftMotor.setPower(1);
        liftMotor.setTargetPosition(-10000);
    }

    public void overrideToggle() {
        overrideLift = !overrideLift;
    }

    public void resetLift() {
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftPos = 8;
    }

    public void clawSwitch() {
        if (!clawExtended) {
            claw.setPosition(0);
            clawExtended = true;
        } else {
            claw.setPosition(0.6);
            clawExtended = false;
        }
    }

    public void bumperMove(int bumperPos) {
        if (bumperPos % 2 == 0) {
            servoLeft.setPosition(0.3);
            servoRight.setPosition(0.7);
        }
        if (bumperPos == 1) {
            servoLeft.setPosition(0.75);
            servoRight.setPosition(0.25);
        }
        if (bumperPos == 3) {
            servoLeft.setPosition(0.05);
            servoRight.setPosition(0.95);
        }
    }

    public void log(Telemetry telemetry) {
        telemetry.addData("Claw position", claw.getPosition());
        telemetry.addData("Lift position", liftMotor.getCurrentPosition());
        telemetry.addData("Lift target", liftMotor.getTargetPosition());
        telemetry.addData("servoLeft", servoLeft.getPosition());
        telemetry.addData("servoRight", servoRight.getPosition());
    }
}

@TeleOp

public class MecanumTeleOpMode extends OpMode {
    public ElapsedTime runtime = new ElapsedTime();
    public double leftStickForward = 0;
    public double leftStickSide = 0;
    public double botSpin = 0;
    public double denominator = 0;
    public double frontLeftPower = 0;
    public double frontRightPower = 0;
    public double rearLeftPower = 0;
    public double rearRightPower = 0;
    boolean pressX = false, pressA = false, pressY = false, pressLbumper = false, pressRbumper = false, pressDpDown = false, pressDpUp = false;
    CRobot robot = new CRobot();

    public void init() {
        robot.init(telemetry, hardwareMap);
        robot.liftMotor.setTargetPosition(0); // sper ca merge desi e in loop
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

        //poate ar trebui sa dai call la functia asta doar o data
        //error : you must set a target position before switching to run to position mode

        ///ATENTIEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEE
        //POSIBIL BUG!!!!!!!!!!!!
        robot.liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //movement robot
        leftStickForward = -this.gamepad1.left_stick_y;
        leftStickSide = this.gamepad1.left_stick_x * 1.1;
        botSpin = this.gamepad1.right_stick_x;
        denominator = Math.max(Math.abs(leftStickForward) + Math.abs(leftStickSide) + Math.abs(botSpin), 1);
        frontLeftPower = (leftStickForward + leftStickSide + botSpin) / denominator;
        rearRightPower = (leftStickForward + leftStickSide - botSpin) / denominator;
        frontRightPower = (leftStickForward - leftStickSide - botSpin) / denominator;
        rearLeftPower = (leftStickForward - leftStickSide + botSpin) / denominator;

        robot.leftFrontMotor.setPower(frontLeftPower);
        robot.rightRearMotor.setPower(rearRightPower);

        robot.rightFrontMotor.setPower(frontRightPower);
        robot.leftRearMotor.setPower(rearLeftPower);
        // control lift

        //lift override (teleop)
        if (this.gamepad1.dpad_up && pressDpUp) {
            robot.overrideToggle();
            pressDpUp = false;
        }
        if (!this.gamepad1.dpad_up) pressDpUp = true;

        if (!robot.overrideLift){robot.runLift(robot.liftPos);}
        else {
            if(this.gamepad1.left_trigger > 0) robot.overrideLiftDown();
            else if (this.gamepad1.right_trigger > 0) robot.overrideLiftUp();
            else robot.liftMotor.setTargetPosition(robot.liftMotor.getCurrentPosition());
        }

        if (this.gamepad1.dpad_down && pressDpDown) {
            robot.powerLift();
            pressDpDown = false;
        }
        if (!this.gamepad1.dpad_down) pressDpDown = true;

        if (this.gamepad1.back) robot.resetLift();

        if (this.gamepad1.y && pressY == true) {
            pressY = false;
            robot.targetLiftUp();
        }
        if (!this.gamepad1.y) pressY = true;

        if (this.gamepad1.a && pressA == true) {
            pressA = false;
            robot.targetLiftDown();
        }
        if (!this.gamepad1.a) pressA = true;

        //bumper
        if (this.gamepad1.right_bumper && pressRbumper == true) {
            robot.bumperPos = (robot.bumperPos + 1) % 4;
            robot.bumperMove(robot.bumperPos);
            pressRbumper = false;
        }
        if (!this.gamepad1.right_bumper) pressRbumper = true;
        //prindere con
        if (this.gamepad1.x && pressX == true) {
            robot.clawSwitch();
            pressX = false;
        }
        if (!this.gamepad1.x) pressX = true;

        // telemetrie
        robot.log(telemetry);
        telemetry.addData("Left Stick Y", leftStickForward);
        telemetry.addData("Left Stick X", leftStickSide);
        telemetry.update();
    }

    @Override
    public void stop() {
    }
}

*/