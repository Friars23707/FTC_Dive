package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.AutonClasses.Claw;
import org.firstinspires.ftc.teamcode.AutonClasses.Slide;

@Autonomous(name = "Specimen Right")
public class RightAuton1 extends LinearOpMode {
    CustomOdometry customOdometry;
    Claw claw;
    Slide slide;
    final private double[] specimenPickup = {3 , -13, 180};
    final private double[] specimenDropoff = {22 , 10, 0};

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


    }
}

