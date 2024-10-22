package org.firstinspires.ftc.teamcode.AutonClasses;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Arm {

    public HardwareMap hwM;

    public  Arm(HardwareMap sent_hwM) {
        hwM = sent_hwM;
    }

    public Action collect() {
        return new Action() {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    initialized = true;
                }

                double vel = 0;
                packet.put("shooterVelocity", vel);
                return vel < 10_000.0;
            }
        };
    }


}
