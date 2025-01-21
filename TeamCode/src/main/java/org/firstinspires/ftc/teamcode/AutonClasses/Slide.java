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

        move(20, 0);
    }

    public void extend(boolean shouldWait) {

        move(1475, -2100);
        if (shouldWait) {
            sleep(2000);
        }

    }

    public void retract(boolean shouldWait) {

        move(20, 0);
        if (shouldWait) {
            sleep(2000);
        }

    }

    public void collection(boolean shouldWait) {

        move(0, -60);
        if (shouldWait) {
            sleep(2000);
        }

    }

    public void highRung(boolean shouldWait) {

        move(1300, -1500);
        if (shouldWait) {
            sleep(2000);
        }

    }

    public void highRungBack(boolean shouldWait) {

        move(600, -800);
        if (shouldWait) {
            sleep(2000);
        }

    }

    private int lastArmPos = 0;

    public void move(int armPos, int slidePos) {
        double armSpeed = lastArmPos <= armPos ? 0.7 : 0.6; //Fast up, slow down
        lastArmPos = armPos;

        leftArm.setTargetPosition(armPos);
        leftArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftArm.setPower(armSpeed);
        
        rightArm.setTargetPosition(armPos);
        rightArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightArm.setPower(armSpeed);

        slide.setTargetPosition(slidePos);
        slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slide.setPower(0.6);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        //I want the sleep function :sob:
    }
}
