package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.AutonClasses.Claw;
import org.firstinspires.ftc.teamcode.AutonClasses.Slide;

// RR-specific imports

@Autonomous(name = "Left_1", group = "Autonomous")
public class Left_1 extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        //Yap yap yap

        //Init most things
        Pose2d initialPose = new Pose2d(11.8, 61.7, Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        Claw claw = new Claw(hardwareMap);
        Slide slide = new Slide(hardwareMap);

        //Create the trajs
        Action goToBucket = drive.actionBuilder(initialPose)
                .lineToYSplineHeading(33, Math.toRadians(0))
                .waitSeconds(3)
                .build();

        Action floorSampleOne = drive.actionBuilder(initialPose)
                .lineToYSplineHeading(33, Math.toRadians(0))
                .waitSeconds(3)
                .build();

        Action floorSampleTwo = drive.actionBuilder(initialPose)
                .lineToYSplineHeading(33, Math.toRadians(0))
                .waitSeconds(3)
                .build();

        Action floorSampleThree = drive.actionBuilder(initialPose)
                .lineToYSplineHeading(33, Math.toRadians(0))
                .waitSeconds(3)
                .build();


        // actions that need to happen on init; for instance, a claw tightening.
        Actions.runBlocking(claw.collect());

        //Custom Wait
        while (!isStopRequested() && !opModeIsActive()) {
            telemetry.addData("Waiting:", "0");
            telemetry.update();
        }

        telemetry.addData("Starting Position", "0");
        telemetry.update();
        waitForStart();

        if (isStopRequested()) return;

        //Run all of the actions
        Actions.runBlocking(
                new SequentialAction(
                        slide.extend(false),
                        goToBucket,
                        claw.eject(),
                        slide.retract(false),
                        floorSampleOne,
                        claw.collect(),
                        goToBucket,
                        claw.eject(),
                        floorSampleTwo,
                        claw.collect(),
                        goToBucket,
                        claw.eject(),
                        floorSampleThree,
                        claw.collect(),
                        goToBucket,
                        claw.eject()
                )
        );

    }
}
