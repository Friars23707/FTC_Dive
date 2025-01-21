package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.AutonClasses.Claw;
import org.firstinspires.ftc.teamcode.AutonClasses.Slide;

@Autonomous(name="Park Ascent")
public class AutoAscent1 extends LinearOpMode {
    CustomOdometry customOdometry;
    Claw claw;
    Slide slide;

    public void runOpMode() throws InterruptedException {
        customOdometry = new CustomOdometry();
        customOdometry.initalize(hardwareMap, telemetry);
        claw = new Claw(hardwareMap);
        slide = new Slide(hardwareMap);

        telemetry.addData("Started", "true");
        telemetry.update();

        waitForStart();


        claw.collect();
        slide.move(900,-50);
        customOdometry.moveTo(-5,50,0);
        customOdometry.moveTo(5,50,0);
        slide.move(1000,-1600);
        sleep(2500);


    }
}
