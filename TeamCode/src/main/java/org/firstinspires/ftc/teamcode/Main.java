package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
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

        telemetry.addData("Running", "true");
        telemetry.update();
        //customOdometry.moveTo(5, 5);
        customOdometry.turn(20);
        telemetry.addData("Ended", "true");
        telemetry.update();
    }

}

