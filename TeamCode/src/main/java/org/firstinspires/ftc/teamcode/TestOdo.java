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
            telemetry.addData("Status", "Initialized");
            telemetry.addData("X offset", odo.getXOffset());
            telemetry.addData("Y offset", odo.getYOffset());
            telemetry.addData("Device Version Number:", odo.getDeviceVersion());
            telemetry.addData("Device Scalar", odo.getYawScalar());
            telemetry.update();
        }
    }
}
