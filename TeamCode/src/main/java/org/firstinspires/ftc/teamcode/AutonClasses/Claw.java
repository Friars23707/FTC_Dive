package org.firstinspires.ftc.teamcode.AutonClasses;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import java.util.List;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class Claw extends LinearOpMode {

    public HardwareMap hwM;
    public Servo claw;
    public Servo wrist;

    public Claw(HardwareMap sent_hwM) {
        hwM = sent_hwM;

        claw = hwM.get(Servo.class, "claw");
        wrist = hwM.get(Servo.class, "wrist");

        wrist.setPosition(0.5);
    }

    public void reset() {
        wrist.setPosition(0.5);
        sleep(500);
        claw.setPosition(0.5);
    }

    public void collect() {

        wrist.setPosition(0.5);
        sleep(500);
        claw.setPosition(0.0);

    }

    public void eject() {

        wrist.setPosition(0.5);
        sleep(500);
        claw.setPosition(1.0);
        reset();

    }


    @Override
    public void runOpMode() throws InterruptedException {
        //I want the sleep function :sob:
    }
}
