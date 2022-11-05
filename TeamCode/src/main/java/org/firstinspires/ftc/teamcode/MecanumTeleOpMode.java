package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.auton.AprilTagDetectionPipeline;
import org.openftc.easyopencv.OpenCvCamera;

@TeleOp

public class MecanumTeleOpMode extends OpMode {
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
    boolean moveX = false, moveA = false, moveY = false, moveLbumper = false, bumperExtended = false, moveRbumper = false, liftUp = false, bumperRight = false, failSafeLift = false, pressDpDown = false, schizoCheck = false;
    int position[] = {-1000, 0, 500, 1000, 3200, 4800, 5600};
    int vPos = 1;

    // String deltaPozStr = ReadWriteFile.readFile(deltaPozFile);
    public void autoSelectLift() {
        if (this.gamepad1.back) {
            liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            vPos = 1;
        }
        if (this.gamepad1.y && moveY == true && vPos < 6) {
            vPos++;
            moveY = false;
            schizoCheck = false;
        }
        if (!this.gamepad1.y) moveY = true;

        if (this.gamepad1.a && moveA == true && vPos > 0) {
            vPos--;
            moveA = false;
            schizoCheck = false;
        }
        if (!this.gamepad1.a) moveA = true;
    }

    public void autoMoveLift() {
        if (this.gamepad1.dpad_down && pressDpDown) {
            failSafeLift = !failSafeLift;
            pressDpDown = false;
        }
        if (!this.gamepad1.dpad_down) pressDpDown = true;

        liftMotor.setTargetPosition(position[vPos]);
        if (liftMotor.getTargetPosition() != liftMotor.getCurrentPosition() && !failSafeLift && !schizoCheck)
            liftMotor.setPower(1);
        else liftMotor.setPower(0);
        if(liftMotor.getTargetPosition() == liftMotor.getCurrentPosition())schizoCheck = true;
    }

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
        servoLeft = hardwareMap.get(Servo.class, "servoLeft");
        servoRight = hardwareMap.get(Servo.class, "servoRight");
        servoLeft.setPosition(0.3);
        servoRight.setPosition(0.7);
        vPos = 1;
        rightFrontMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        rightRearMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        leftFrontMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRearMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        liftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        telemetry.update();
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

        // control lift
        autoSelectLift();
        autoMoveLift();
        telemetry.addData("Lift target", liftMotor.getTargetPosition());

        /*
        if(this.gamepad1.y && moveY==true && vPos<4){vPos++;moveY=false;}
        if(!this.gamepad1.y)moveY=true;

        if(this.gamepad1.a && moveA==true && vPos>0){vPos--;moveA=false;}
        if(!this.gamepad1.a)moveA=true;

        liftMotor.setTargetPosition(position[vPos] - deltaPoz);
        if(this.gamepad1.right_trigger > 0 && liftMotor.getCurrentPosition()!=liftMotor.getTargetPosition()){liftMotor.setPower(1);}
        else
        if(this.gamepad1.left_trigger > 0 && liftMotor.getCurrentPosition()!=liftMotor.getTargetPosition()){vPos=0;liftMotor.setPower(1);}
        else liftMotor.setPower(0);

        if(this.gamepad1.y && moveY==true){
            if(liftUp) {
                liftMotor.setTargetPosition(-10000);
                liftUp=false;
            }
            else {
                liftMotor.setTargetPosition(10000);
                liftUp=true;
            }
            moveY=false;
        }
        if(!this.gamepad1.y)moveY=true;
        */


        if (this.gamepad1.right_bumper && moveRbumper == true) {
            if (bumperExtended) {
                servoLeft.setPosition(0.75);
                servoRight.setPosition(0.25);
                bumperExtended = false;
            } else {
                servoLeft.setPosition(0.1);
                servoRight.setPosition(0.9);
                bumperExtended = true;
            }
            moveRbumper = false;
        }
        if (!this.gamepad1.right_bumper) moveRbumper = true;
       /* if(this.gamepad1.a && moveA==true){liftMotor.setTargetPosition(10000);moveA=false;}
        if(!this.gamepad1.a)moveA=true;*/

        //  liftMotor.setTargetPosition(position[vPos] - deltaPoz);
        if (this.gamepad1.right_trigger > 0) {
            liftMotor.setPower(1);
        } else liftMotor.setPower(0);


        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        telemetry.addData("Lift encoder", liftMotor.getCurrentPosition());
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
        telemetry.addData("servoLeft", servoLeft.getPosition());
        telemetry.addData("servoRight", servoRight.getPosition());

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

