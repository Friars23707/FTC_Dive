package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.CustomOdometry;

public class Main extends OpMode {
    CustomOdometry customOdometry;

    @Override
    public void init() {
        customOdometry = new CustomOdometry();
        customOdometry.initalize();
    }

    public void loop() {
        customOdometry.moveTo(30, 40);
        customOdometry.turn(Math.PI);
    }
}

