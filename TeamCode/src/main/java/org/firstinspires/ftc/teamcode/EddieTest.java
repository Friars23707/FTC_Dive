package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
@TeleOp(name="Eddie Test",group="TeleOps")
public class EddieTest extends OpMode{
    public DcMotor lift;

    @Override
    public void init() {
        lift = hardwareMap.get(DcMotor.class,"lift");
    }

    @Override
    public void loop() {
        lift.setPower(gamepad1.left_stick_y * 75);

    }
}
