package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp

public class LearningOpMode extends LinearOpMode {
    private DcMotor leftMotor;
    private DcMotor rightMotor;

    @Override
    public void runOpMode() {
        leftMotor = hardwareMap.get(DcMotor.class, "left");
        rightMotor = hardwareMap.get(DcMotor.class, "right");
        double tgtPowerY = 0;
        double tgtPowerX = 0;
        waitForStart();

        while(opModeIsActive()) {
            tgtPowerY = -this.gamepad1.left_stick_y;
            tgtPowerX = this.gamepad1.left_stick_x;

            if(Math.abs(tgtPowerY) > 0.1 || Math.abs(tgtPowerX) > 0.1) {
                rightMotor.setPower(tgtPowerY + tgtPowerX);
                leftMotor.setPower(tgtPowerY - tgtPowerX);
            }

            telemetry.addData("target power Y", tgtPowerY);
            telemetry.addData("left motor power", leftMotor.getPower());
            telemetry.addData("right motor power", rightMotor.getPower());
            telemetry.update();
        }
    }
}
