//package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class CRobot {
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
        rightFrontMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rightRearMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFrontMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        leftRearMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        liftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        telemetry.update();
    }

    boolean clawExtended = false;
    boolean liftPower = true;
    boolean overrideLift = true;
    int position[] = {-7420, -5700, -4600, -3300, -2500, -1100, -600, 0, 600, 1500, 2500, 3300, 4600, 5700, 7430};
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