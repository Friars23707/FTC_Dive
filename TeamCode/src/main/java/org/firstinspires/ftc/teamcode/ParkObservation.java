package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.AutonClasses.Claw;
import org.firstinspires.ftc.teamcode.AutonClasses.Slide;

@Autonomous(name = "Park Observaion")
public class ParkObservation extends LinearOpMode {
    CustomOdometry customOdometry;

    Claw claw;
    Slide slide;

    @Override
    public void runOpMode() throws InterruptedException {
        customOdometry = new CustomOdometry();
        customOdometry.initalize(hardwareMap, telemetry);
        claw = new Claw(hardwareMap);
        slide = new Slide(hardwareMap);

        telemetry.addData("Started", "true");
        telemetry.update();

        waitForStart();

        claw.collect();
        customOdometry.moveTo(0,-40,0);

    }



}