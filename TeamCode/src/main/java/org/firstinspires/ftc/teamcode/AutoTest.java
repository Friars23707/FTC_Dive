package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.AutonClasses.Slide;

@Autonomous(name = "AutoTest", group = "Competition")
public class AutoTest extends LinearOpMode {
    private CustomOdometry odometry;
    private Slide slide;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize hardware
        odometry = new CustomOdometry();
        slide = new Slide(hardwareMap);  // Remove if not using slide

        // Initialize odometry system
        odometry.initialize(hardwareMap, telemetry);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        // Main autonomous sequence
        if (opModeIsActive()) {
            // First movement: 5 inches X, 5 inches Y, 45° heading
            odometry.moveTo(5.0, 5.0, 45.0);

            // Second movement: Return to origin with -45° heading
            odometry.moveTo(0.0, 0.0, -45.0);
        }

        // Add any end-of-auto cleanup here
        telemetry.addData("Status", "Complete");
        telemetry.update();
    }
}

