package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.AutonClasses.Claw;
import org.firstinspires.ftc.teamcode.AutonClasses.Slide;

@Autonomous(name = "Park Test")
public class ClawTesting extends LinearOpMode {
    CustomOdometry customOdometry;
    Claw claw;
    Slide slide;
    final private double[] bucketLocation = {45 , -3, 37};
    final private double[] parkLocation = {20, -53, 180};
    final private double sampleY = -37;

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

        customOdometry.moveTo(bucketLocation[0]-10, bucketLocation[1]-10, 0);
        slide.retract(false);
        sleep(200);

        slide.move(900, -500);
        customOdometry.moveTo(parkLocation[0]+10, parkLocation[1], parkLocation[2]);
        customOdometry.moveTo(parkLocation[0]+14, parkLocation[1], parkLocation[2]);

        /*
        customOdometry.moveTo(33, sampleY, 0);
        slide.collection(false);
        claw.collect();
        customOdometry.moveTo(50.3, sampleY, 0);
        sleep(500);
        customOdometry.moveTo(bucketLocation[0]-10, bucketLocation[1]-10, bucketLocation[2]);
        customOdometry.moveTo(bucketLocation[0], bucketLocation[1], bucketLocation[2]);
        telemetry.addData("MOVETOFINISHED", true);
        telemetry.update();
        slide.extend(false);
        claw.eject();
        sleep(500);
        slide.retract(false);
        customOdometry.moveTo(bucketLocation[0]-10, bucketLocation[1]-10, 0);

         */



    }
}

