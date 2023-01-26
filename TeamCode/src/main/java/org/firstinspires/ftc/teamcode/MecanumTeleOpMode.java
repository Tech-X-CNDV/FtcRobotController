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

    private Servo servo1;
    private Servo servo2;

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
    boolean clawExtended = false;
    boolean moveX = false, moveA = false, moveY = false, moveLbumper = false, moveRbumper;
    int position[]={500,700,1800,3200,4800};
    int vPos = 0;
    int targetPosition;
    String deltaPozStr = ReadWriteFile.readFile(deltaPozFile);
    int deltaPoz =Integer.valueOf(deltaPozStr);
    int deltaPozAux;
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
        servo1 = hardwareMap.get(Servo.class, "servo1");
        servo2 = hardwareMap.get(Servo.class, "servo2");
        servo1.setPosition(0.1);servo2.setPosition(0.1);
        liftMotor.setTargetPosition(0 + deltaPoz);
        webcam = hardwareMap.get(WebcamName.class, "camera");
       // camera = OpenCvCameraFactory.getInstance().createWebcam(webcam, cameraMonitorViewId);

        deltaPoz = deltaPoz + Integer.valueOf(deltaPozStr);

        rightFrontMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        rightRearMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        leftFrontMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRearMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        liftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        //liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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

        // control lift

        telemetry.addData("Lift target", liftMotor.getTargetPosition());

        if(this.gamepad1.y && moveY==true && vPos<4){vPos++;moveY=false;}
        if(!this.gamepad1.y)moveY=true;

        if(this.gamepad1.a && moveA==true && vPos>0){vPos--;moveA=false;}
        if(!this.gamepad1.a)moveA=true;

        if(this.gamepad1.right_trigger > 0 && liftMotor.getCurrentPosition()!=liftMotor.getTargetPosition()){liftMotor.setTargetPosition(position[vPos] - deltaPoz);liftMotor.setPower(1);}
        else
        if(this.gamepad1.left_trigger > 0 && liftMotor.getCurrentPosition()!=liftMotor.getTargetPosition()){liftMotor.setTargetPosition(position[0]-deltaPoz);liftMotor.setPower(1);vPos=0;}
        else liftMotor.setPower(0);

        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        telemetry.addData("Lift encoder", liftMotor.getCurrentPosition());
        telemetry.addData("deltaPoz", deltaPoz);
        telemetry.addData("deltaPozAux", deltaPozAux);
        //if (liftMotor.getCurrentPosition() == targetPosition)
          //  telemetry.addLine("EXTENDED");
        //sevo conuri
        if(this.gamepad1.left_bumper && moveLbumper) {
                servo1.setPosition(0.1);
                servo2.setPosition(0.1);
                moveLbumper = false;
        }
        else moveLbumper = true;
        if(this.gamepad1.right_bumper && moveRbumper) {
            servo1.setPosition(0.6);
            servo2.setPosition(0.6);
            moveRbumper = false;
        }
        else moveRbumper = true;
        //prindere gheara
        if (this.gamepad1.x && moveX==true){
            if(!clawExtended){
                telemetry.addLine("Claw EXTENDED");
                claw.setPosition(0.2);
                clawExtended = true;
            } else {
                telemetry.addLine("Claw RETRACTED");
                claw.setPosition(0.5);
                clawExtended = false;
            }
            moveX=false;
        }
        if(!this.gamepad1.x)moveX=true;
        telemetry.addData("Servo Position", claw.getPosition());

        leftFrontMotor.setPower(frontLeftPower);
        rightRearMotor.setPower(rearRightPower);

        rightFrontMotor.setPower(frontRightPower);
        leftRearMotor.setPower(rearLeftPower);

        deltaPozAux = liftMotor.getCurrentPosition();
        ReadWriteFile.writeFile(deltaPozFile,String.valueOf(deltaPozAux));
        telemetry.update();
    }

    @Override
    public void stop(){}
}

