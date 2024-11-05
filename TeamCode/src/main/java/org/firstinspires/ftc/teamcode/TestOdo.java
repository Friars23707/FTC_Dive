package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous
public class TestOdo extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        GoBildaPinpointDriver odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");

        waitForStart();

        while (!isStopRequested()) {
            telemetry.addData("x: ", odo.getPosX());
            telemetry.addData("y: ", odo.getPosY());
            telemetry.update();
        }
    }
}
