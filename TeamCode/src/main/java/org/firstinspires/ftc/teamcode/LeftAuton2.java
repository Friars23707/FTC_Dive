package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.AutonClasses.Claw;
import org.firstinspires.ftc.teamcode.AutonClasses.Slide;

@Autonomous(name = "Bucket Left")
public class LeftAuton2 extends LinearOpMode {
    CustomOdometry customOdometry;
    Claw claw;
    Slide slide;
    final private double[] bucketLocation = {44 , -4, 37};
    final private double[] parkLocation = {20, -53, 0};
    final private double sampleY = -37;

    @Override
    public void runOpMode() throws InterruptedException {

        customOdometry = new CustomOdometry();
        customOdometry.initialize(hardwareMap, telemetry);
        claw = new Claw(hardwareMap);
        slide = new Slide(hardwareMap);

        telemetry.addData("Started", "true");
        telemetry.update();

        waitForStart();


        slide.extend(false);
        claw.collect();

        customOdometry.moveTo(bucketLocation[0], bucketLocation[1], bucketLocation[2]);
        telemetry.addData("MOVETOFINISHED", true);
        telemetry.update();
        claw.eject();
        sleep(500);
        customOdometry.moveTo(bucketLocation[0]-10, bucketLocation[1]-10, 0);
        slide.retract(false);

        customOdometry.moveTo(25, -15, 0);
        customOdometry.moveTo(25, sampleY, 0);
        slide.collection(false);
        claw.collect();
        customOdometry.moveTo(33, sampleY, 0);
        sleep(500);
        slide.extend(false);
        customOdometry.moveTo(bucketLocation[0]-10, bucketLocation[1]-10, 0);
        customOdometry.moveTo(bucketLocation[0]+1, bucketLocation[1], bucketLocation[2]-8);
        telemetry.addData("MOVETOFINISHED", true);
        telemetry.update();
        claw.eject();
        sleep(500);
        customOdometry.moveTo(bucketLocation[0]-10, bucketLocation[1]-10, 0);
        slide.retract(false);
        sleep(200);

        customOdometry.moveTo(parkLocation[0]+10, parkLocation[1], parkLocation[2]);
        customOdometry.moveTo(parkLocation[0]+10, parkLocation[1], 180);
        slide.move(900, -2100);
        sleep(2500);

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

