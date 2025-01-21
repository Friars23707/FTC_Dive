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
    final private double[] specimenPickup = {3 , -20, 0};
    final private double[] specimenDropoff = {23 , 20, 0};

    @Override
    public void runOpMode() throws InterruptedException {

        customOdometry = new CustomOdometry();
        customOdometry.initalize(hardwareMap, telemetry);
        claw = new Claw(hardwareMap);
        slide = new Slide(hardwareMap);

        telemetry.addData("Started", "true");
        telemetry.update();

        waitForStart();


        slide.highRung(false);
        claw.side();

        customOdometry.moveTo(specimenDropoff[0], specimenDropoff[1], specimenDropoff[2]);
        telemetry.addData("MOVETOFINISHED", true);
        telemetry.update();
        slide.highRungBack(true);

        slide.collection(false);
        claw.reset();
        customOdometry.moveTo(specimenPickup[0], specimenPickup[1], specimenPickup[2]);

    }
}

