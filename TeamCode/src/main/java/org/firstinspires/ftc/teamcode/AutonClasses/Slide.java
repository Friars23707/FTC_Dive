package org.firstinspires.ftc.teamcode.AutonClasses;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Slide extends LinearOpMode {

    public HardwareMap hwM;
    public DcMotor leftArm;
    public DcMotor rightArm;
    public DcMotor slide;

    public Slide(HardwareMap sent_hwM) {
        hwM = sent_hwM;

        leftArm = hwM.get(DcMotor.class,"arm_left");
        rightArm = hwM.get(DcMotor.class,"arm_right");
        slide = hwM.get(DcMotor.class,"slide");

        leftArm.setDirection(DcMotor.Direction.REVERSE);
        leftArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rightArm.setDirection(DcMotor.Direction.FORWARD);
        rightArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        slide.setDirection(DcMotor.Direction.FORWARD);
        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        move(200, 0);
    }

    public void extend(boolean shouldWait) {

        move(1000, -200);
        if (shouldWait) {
            sleep(2000);
        }

    }

    public void retract(boolean shouldWait) {

        move(200, 0);
        if (shouldWait) {
            sleep(2000);
        }

    }

    public void move(int armPos, int slidePos) {
        leftArm.setTargetPosition(armPos);
        leftArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftArm.setPower(0.3);
        rightArm.setTargetPosition(armPos);
        rightArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightArm.setPower(0.3);
        slide.setTargetPosition(slidePos);
        slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slide.setPower(0.3);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        //I want the sleep function :sob:
    }
}
