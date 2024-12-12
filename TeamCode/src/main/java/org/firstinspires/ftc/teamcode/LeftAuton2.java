package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.AutonClasses.Claw;
import org.firstinspires.ftc.teamcode.AutonClasses.Slide;

@Autonomous(name = "W Autos")
public class LeftAuton2 extends LinearOpMode {
    CustomOdometry customOdometry;
    Claw claw;
    Slide slide;

    final private int[] bucketLocation = {45, -3};
    final private int sampleY = -36;

    @Override
    public void runOpMode() throws InterruptedException {

        customOdometry = new CustomOdometry();
        customOdometry.initalize(hardwareMap, telemetry);
        claw = new Claw(hardwareMap);
        slide = new Slide(hardwareMap);

        telemetry.addData("Started", "true");
        telemetry.update();

        waitForStart();

        customOdometry.moveTo(26, sampleY);
        claw.collect();
        customOdometry.moveTo(bucketLocation[0], bucketLocation[1]);
        slide.extend(true);
        claw.eject();
        slide.retract(false);

        customOdometry.moveTo(37, sampleY);
        claw.collect();
        customOdometry.moveTo(bucketLocation[0], bucketLocation[1]);
        slide.extend(true);
        claw.eject();
        slide.retract(false);

        customOdometry.moveTo(48, sampleY);
        claw.collect();
        customOdometry.moveTo(bucketLocation[0], bucketLocation[1]);
        slide.extend(true);
        claw.eject();
        slide.retract(false);


    }
}

