package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.AutonClasses.Claw;
import org.firstinspires.ftc.teamcode.AutonClasses.Slide;

@Autonomous
public class LeftAuton3 extends LinearOpMode {

    CustomOdometry customOdometry;
    Claw claw;
    Slide slide;
    final private double[] bucketLocation = {44 , -4, 35};
    final private double sampleY = -36.25;
    @Override
    public void runOpMode() throws InterruptedException {

        customOdometry = new CustomOdometry();
        customOdometry.initalize(hardwareMap, telemetry);
        claw = new Claw(hardwareMap);
        slide = new Slide(hardwareMap);

        telemetry.addData("Started", "true");
        telemetry.update();

        waitForStart();


        slide.extend(false);
        claw.collect();

        sleep(2000);

        claw.eject();
        sleep(2000);

    }
}
