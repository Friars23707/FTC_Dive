package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public class CustomOdometry extends LinearOpMode {
    GoBildaPinpointDriver odo;
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;

    public Telemetry telem;

    public void initalize(HardwareMap hwm, Telemetry tm) {
        telem = tm;
        odo = hwm.get(GoBildaPinpointDriver.class, "odo");
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        odo.resetPosAndIMU();

        leftFrontDrive = hwm.get(DcMotor.class, "front_left");
        leftBackDrive = hwm.get(DcMotor.class, "back_left");
        rightBackDrive = hwm.get(DcMotor.class, "back_right");
        rightFrontDrive = hwm.get(DcMotor.class, "front_right");

        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        resetRuntime();
    }


    public void moveTo(double x, double y) {
        Pose2D position;
        do {
            position = odo.getPosition();
            double currentX = position.getX(DistanceUnit.INCH);
            double currentY = position.getY(DistanceUnit.INCH);

            telem.addData("Target: ", "{X: %.3f, Y: %.3f}", x, y);
            telem.addData("Current: ", "{X: %.3f, Y: %.3f}", currentX, currentY);
            telem.update();

            double axial = x - currentX;
            double lateral = y - currentY;

            double leftFrontPower = axial + lateral;
            double rightFrontPower = axial - lateral;
            double leftBackPower = axial - lateral;
            double rightBackPower = axial + lateral;

            double max;
            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));

            if (max > 1.0) {
                leftFrontPower /= max;
                rightFrontPower /= max;
                leftBackPower /= max;
                rightBackPower /= max;
            }

            leftFrontDrive.setPower(leftFrontPower);
            rightFrontDrive.setPower(rightFrontPower);
            leftBackDrive.setPower(leftBackPower);
            rightBackDrive.setPower(rightBackPower);

            odo.update();
        } while ((Math.abs(x - position.getX(DistanceUnit.INCH)) > 0.1 || Math.abs(y - position.getY(DistanceUnit.INCH)) > 0.1) && !isStopRequested()); // Assuming a threshold for reaching target
    }

    public void turn(double angle) {
        Pose2D position = odo.getPosition();
        double currentAngle = position.getHeading(AngleUnit.DEGREES);
        double targetAngle = currentAngle + angle;

        while (((currentAngle < targetAngle) || (currentAngle < 180-targetAngle)) && !isStopRequested()) {
            telem.addData("Target: ", targetAngle);
            telem.addData("Current: ", odo.getPosition().getHeading(AngleUnit.DEGREES));
            telem.update();

            double error = targetAngle - odo.getPosition().getHeading(AngleUnit.DEGREES);
            double power = 0.3 * (error / Math.abs(angle)); // Adjust the constant (0.5) to fine-tune the power scaling

            if (power > 0.4) power = 0.4; // Cap the maximum power
            if (power < -0.4) power = -0.4; // Cap the minimum power

            leftFrontDrive.setPower(-power);
            rightFrontDrive.setPower(power);
            leftBackDrive.setPower(-power);
            rightBackDrive.setPower(power);

            odo.update();
        }

        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
    }


    @Override
    public void runOpMode() throws InterruptedException {}
}


