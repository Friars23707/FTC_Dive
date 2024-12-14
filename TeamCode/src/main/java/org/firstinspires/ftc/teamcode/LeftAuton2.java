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

    final private int[] bucketLocation = {48 , -2, 20};
    final private int sampleY = -37;

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
        claw.reset();

        customOdometry.moveTo(bucketLocation[0], bucketLocation[1], bucketLocation[2]);
        claw.eject();
        slide.retract(false);

        customOdometry.moveTo(27, -15, 0);
        customOdometry.moveTo(25, sampleY, 0);
        slide.collection(false);
        claw.collect();
        customOdometry.moveTo(29, sampleY, 0);
        sleep(2000);
        slide.extend(false);
        claw.reset();
        customOdometry.moveTo(bucketLocation[0], bucketLocation[1], bucketLocation[2]);
        claw.eject();
        slide.retract(false);

        customOdometry.moveTo(36, sampleY, 0);
        slide.collection(false);
        claw.collect();
        customOdometry.moveTo(40, sampleY, 0);
        sleep(2000);
        slide.extend(false);
        claw.reset();
        customOdometry.moveTo(bucketLocation[0], bucketLocation[1], bucketLocation[2]);
        claw.eject();
        slide.retract(false);

/* RE ADD IF WE HAVE THE TIME FOR IT
        customOdometry.moveTo(46, sampleY, 0);
        slide.collection(false);
        claw.collect();
        customOdometry.moveTo(50, sampleY, 0);
        sleep(2000);
        claw.reset();
        customOdometry.moveTo(bucketLocation[0], bucketLocation[1]-15, bucketLocation[2]);
        slide.extend(true);
        customOdometry.moveTo(bucketLocation[0], bucketLocation[1], bucketLocation[2]);
        claw.eject();
        slide.retract(false);
*/

    }
}

