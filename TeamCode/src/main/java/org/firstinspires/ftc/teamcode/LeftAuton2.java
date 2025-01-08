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

        customOdometry.moveTo(bucketLocation[0], bucketLocation[1], bucketLocation[2]);
        telemetry.addData("MOVETOFINISHED", true);
        telemetry.update();
        claw.eject();
        customOdometry.moveTo(bucketLocation[0]-10, bucketLocation[1]-10, 0);
        slide.retract(false);

        customOdometry.moveTo(27, -15, 0);
        customOdometry.moveTo(25, sampleY, 0);
        slide.collection(false);
        claw.collect();
        customOdometry.moveTo(30, sampleY, 0);
        sleep(500);
        slide.extend(false);
        customOdometry.moveTo(bucketLocation[0], bucketLocation[1], bucketLocation[2]);
        telemetry.addData("MOVETOFINISHED", true);
        telemetry.update();
        claw.eject();
        customOdometry.moveTo(bucketLocation[0]-10, bucketLocation[1]-10, 0);
        slide.retract(false);

        customOdometry.moveTo(36, sampleY, 0);
        slide.collection(false);
        claw.collect();
        customOdometry.moveTo(45   , sampleY, 0);
        sleep(500);
        slide.extend(false);
        customOdometry.moveTo(bucketLocation[0], bucketLocation[1], bucketLocation[2]);
        telemetry.addData("MOVETOFINISHED", true);
        telemetry.update();
        claw.eject();
        slide.retract(false);
        customOdometry.moveTo(bucketLocation[0]-10, bucketLocation[1]-10, 0);



    }
}

