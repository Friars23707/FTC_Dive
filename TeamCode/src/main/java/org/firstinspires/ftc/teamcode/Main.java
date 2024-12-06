package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.CustomOdometry;

@Autonomous
public class Main extends LinearOpMode {
    CustomOdometry customOdometry;

    @Override
    public void runOpMode() throws InterruptedException {

        customOdometry = new CustomOdometry();
        customOdometry.initalize(hardwareMap, telemetry);
        telemetry.addData("Started", "true");
        telemetry.update();

        waitForStart();
        while (true) {
            //telemetry.addData("Running", "true");
            //telemetry.update();
            //customOdometry.moveTo(5, 5);
            //customOdometry.turn(180);
            //telemetry.addData("Ended", "true");
            telemetry.addData("Heading: ", customOdometry.odo.getHeading(AngleUnit.DEGREES));
            telemetry.update();

        }
    }
}

