package org.firstinspires.ftc.teamcode;

import android.os.DropBoxManager;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;


import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.internal.camera.delegating.DelegatingCaptureSequence;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
//import org.firstinspires.ftc.teamcode.auton.AprilTagDetectionPipeline;
import org.openftc.easyopencv.OpenCvCamera;

class CRobot {
    public DcMotor leftFrontMotor;
    public DcMotor rightFrontMotor;
    public DcMotor leftRearMotor;
    public DcMotor rightRearMotor;
    public DcMotor liftMotor;
    public Servo claw;
    public Servo servoLeft;
    public Servo servoRight;
    public Servo servoRotator;


    public TouchSensor liftSensor;
    public ColorSensor colorSensor;
    public DistanceSensor distanceSensor;

    public void init(Telemetry telemetry, HardwareMap hardwareMap) {
        telemetry.addData("Status", "Initialized");
        tared = false;
        hasCone = false;
        keepDistance = false;
        colorSensor = hardwareMap.get(ColorSensor.class, "distanceSensor");
        distanceSensor = hardwareMap.get(DistanceSensor.class, "distanceSensor");
        leftFrontMotor = hardwareMap.get(DcMotor.class, "leftFrontMotor");
        rightFrontMotor = hardwareMap.get(DcMotor.class, "rightFrontMotor");
        leftRearMotor = hardwareMap.get(DcMotor.class, "leftRearMotor");
        rightRearMotor = hardwareMap.get(DcMotor.class, "rightRearMotor");
        liftSensor = hardwareMap.get(TouchSensor.class, "touchSensor");
        liftMotor = hardwareMap.get(DcMotor.class, "liftMotor");
        claw = hardwareMap.get(Servo.class, "clawServo");
        claw.setPosition(0.6);
        liftPos = 8;
        servoLeft = hardwareMap.get(Servo.class, "servoLeft");
        servoRight = hardwareMap.get(Servo.class, "servoRight");
        servoRotator = hardwareMap.get(Servo.class, "servoRotator");
        bumperMove(1);
        rightFrontMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rightRearMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFrontMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        leftRearMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        liftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        telemetry.update();
    }

    boolean armedBumper = false;
    boolean hasCone = false;
    boolean tared = false;
    boolean clawExtended = false;
    boolean liftPower = true;
    boolean overrideLift = true;
    boolean frontClaw = true;
    boolean keepDistance = false;
    int position[] = {-7420, -5700, -4600, -3300, -2500, -1100, -600, 0, 600, 2300, 3900, 5260};
    int stackpos[] = {1450,825,600,420,220,60};
    public int liftPos = 8, bumperPos = 0;
    //60 - 220 - 450 600 - 940 - 1400
    public void runLiftStack(int liftPos) {
        liftMotor.setTargetPosition(stackpos[liftPos]);
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void targetLiftUp() {
        if (liftPos < 11) {
            liftPos++;
        }
    }

    public void targetLiftDown() {
        if (tared && liftPos >= 8)
            liftPos--;
        else if (liftPos > 0) {
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
            liftMotor.setTargetPosition(5260);
    }

    public void overrideLiftDown() {
            liftMotor.setTargetPosition(-5260);
    }

    public void liftTaring() {
        if (liftSensor.isPressed() && !tared) {
            tared = true;
            resetLift();
        }
    }

    public void overrideToggle() {
        overrideLift = !overrideLift;
    }

    public void resetLift() {
        liftMotor.setPower(0);
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotor.setTargetPosition(0);
        liftPos = 8;
    }

    public void clawSwitch() {
        if (hasCone)hasCone = false;
        if (!clawExtended) {
            claw.setPosition(0);
            clawExtended = true;
        } else {
            claw.setPosition(0.6);
            clawExtended = false;
        }
    }

    public void automaticCone() {
        if (armedBumper)
            bumperMove(3);
        if (!hasCone && tared && armedBumper) {
            switch ((int) distanceSensor.getDistance(DistanceUnit.CM) / 5) {
                case 0:
                        bumperMove(2);
                    if (distanceSensor.getDistance(DistanceUnit.CM) <= 1.5 && colorSensor.blue() > 20) {
                        liftPos = 7;
                        runLift(7);
                        clawExtended = true;
                        bumperMove(3);
                        clawSwitch();
                        while(liftMotor.getCurrentPosition()>10){}
                        clawSwitch();
                        bumperMove(3);
                        hasCone = true;
                        armedBumper = false;
                    }
                    break;
                case 1:
                    if(distanceSensor.getDistance(DistanceUnit.CM)<=8.5){liftPos = 8;runLift(8);}
                    break;

            }
        }
        if (!armedBumper && liftMotor.getCurrentPosition() >= position[9] && hasCone)
            bumperMove(1);
    }

    public boolean junctionPositioning(){
        if(keepDistance && liftMotor.getCurrentPosition() >= position[8]){
            if (distanceSensor.getDistance(DistanceUnit.CM) < 2 )
            {
                return true;}
        }
        return false;
    }

    public void rotateClaw(){
        if(liftMotor.getCurrentPosition() >= (position[9]+position[10])/2){
            if(frontClaw){
                frontClaw = false;
                servoRotator.setPosition(1);
            }
            else{
                frontClaw = true;
                servoRotator.setPosition(0);
            }
        }
    }

    public void bumperArm() {
        armedBumper= !armedBumper;
    }

    public void bumperMove(int bumperPos) {
        if (bumperPos % 2 == 0) {
            servoLeft.setPosition(0.45);
            servoRight.setPosition(0.55);
        }
        if (bumperPos == 1) {
            servoLeft.setPosition(0.70);
            servoRight.setPosition(0.30);
        }
        if (bumperPos == 3) {
            servoLeft.setPosition(0.20);
            servoRight.setPosition(0.80);
        }
    }

    public void log(Telemetry telemetry) {
        telemetry.addData("Distance", distanceSensor.getDistance(DistanceUnit.CM));
        telemetry.addData("Claw position", claw.getPosition());
        telemetry.addData("Lift position", liftMotor.getCurrentPosition());
        telemetry.addData("Lift target", liftMotor.getTargetPosition());
        telemetry.addData("servoLeft", servoLeft.getPosition());
        telemetry.addData("servoRight", servoRight.getPosition());
    }
}

@TeleOp

public class Mecanum extends OpMode {
    public ElapsedTime runtime = new ElapsedTime();
    public double leftStickForward = 0;
    public double leftStickSide = 0;
    public double botSpin = 0;
    public double denominator = 0;
    public double frontLeftPower = 0;
    public double frontRightPower = 0;
    public double rearLeftPower = 0;
    public double rearRightPower = 0;
    boolean pressX = false, pressA = false, pressY = false, pressLbumper = false, pressRbumper = false, pressDpDown = false, pressDpUp = false, pressDpLeft = false, pressBumperG1 = false;
    CRobot robot = new CRobot();

    public void init() {
        robot.init(telemetry, hardwareMap);
        robot.liftMotor.setTargetPosition(0); // sper ca merge desi e in loop
        robot.liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
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

        //movement robot
        if(robot.junctionPositioning()) {
            leftStickForward = .5;
        }
            else
            leftStickForward = leftStickForward - this.gamepad2.left_stick_y / 0.01;
        if (this.gamepad2.left_stick_y == 0) leftStickForward = 0;
        leftStickSide = this.gamepad2.left_stick_x * 1.1;
        botSpin = this.gamepad2.right_stick_x;
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

        if (this.gamepad2.right_bumper && pressBumperG1) {
            robot.keepDistance = !robot.keepDistance;
            pressBumperG1 = false;
        }
        if (!this.gamepad2.right_bumper) pressBumperG1 = true;

        if (!robot.overrideLift){robot.runLift(robot.liftPos);}
        else {
            if (robot.liftPower) robot.liftMotor.setPower(1);
            else robot.liftMotor.setPower(0);
            if (this.gamepad1.left_trigger > 0) robot.overrideLiftDown();
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

        //calibrare lift
        robot.liftTaring();
        robot.automaticCone();
        //bumper
        if (this.gamepad1.right_bumper && pressRbumper == true) {
            robot.bumperArm();
            pressRbumper = false;
        }
        if (!this.gamepad1.right_bumper) pressRbumper = true;
        //prindere con
        if (this.gamepad1.x && pressX == true) {
            robot.clawSwitch();
            pressX = false;
        }
        if (!this.gamepad1.x) pressX = true;
        //rotire gheara
        if(this.gamepad1.dpad_left && pressDpLeft == true){
            robot.rotateClaw();
            pressDpLeft = false;
        }
        if(!this.gamepad1.dpad_left) pressDpLeft = true;
        // telemetrie
        robot.log(telemetry);
        telemetry.addData("Left Stick Y", leftStickForward);
        telemetry.addData("Left Stick X", leftStickSide);
        telemetry.addData("tar", robot.tared);
        telemetry.addData("blue", robot.colorSensor.blue());
        telemetry.addData("armed", robot.armedBumper);
        telemetry.update();
    }

    @Override
    public void stop() {
    }
}