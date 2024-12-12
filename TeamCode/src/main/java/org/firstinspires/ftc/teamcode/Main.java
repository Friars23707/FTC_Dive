package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.AutonClasses.Slide;
import org.firstinspires.ftc.teamcode.CustomOdometry;

@Autonomous
public class Main extends LinearOpMode {
    CustomOdometry customOdometry;
    Slide slide;

    @Override
    public void runOpMode() throws InterruptedException {

        customOdometry = new CustomOdometry();
        slide = new Slide(hardwareMap);
        customOdometry.initalize(hardwareMap, telemetry);
        telemetry.addData("Started", "true");
        telemetry.update();

        waitForStart();
        while (!isStopRequested()) {
            /*customOdometry.turnTo(45);
            sleep(600);
            customOdometry.moveTo(5, 5);
            sleep(600);
            customOdometry.moveTo(0, 0);
            sleep(600);
            customOdometry.turnTo(0);
            sleep(600);
            customOdometry.moveTo(-5, -5);
            sleep(600);*/

            customOdometry.moveToWithHeading(5, 5, 45);
            sleep(4000);
            customOdometry.moveToWithHeading(0, 0, -45);
            sleep(4000);
        }

    }
}

