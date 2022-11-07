/*
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.MecanumTeleOpMode;

public class functiiRobot extends MecanumTeleOpMode {
    boolean clawExtended = false;
    boolean liftPower = true;
    int position[] = {-1000, 0, 500, 1000, 2400, 3200, 4000, 4800, 5600, 6400, 7200};
    public int liftPos = 1, bumperPos = 0;
    public void targetLiftUp() {
        if(liftPos<10){
            liftPos++;
        }
    }
    public void targetLiftDown(){
        if(liftPos>0) {
            liftPos--;
        }
    }

    public void powerLift() {
        liftPower = !liftPower;
    }
    public void runLift(int liftPos)
    {
        liftMotor.setTargetPosition(position[liftPos]);
        if (liftPower)liftMotor.setPower(1);
        else liftMotor.setPower(0);
    }
    public void resetLift()
    {
            liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            liftPos = 1;
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
    public void bumperMove(int bumperPos)
    {
        if (bumperPos % 2 == 0) {
            servoLeft.setPosition(0.3);
            servoRight.setPosition(0.7);
        }
        if (bumperPos == 1) {
            servoLeft.setPosition(0.75);
            servoRight.setPosition(0.25);
        }
        if (bumperPos == 3) {
            servoLeft.setPosition(0);
            servoRight.setPosition(1);
        }
    }
}*/
