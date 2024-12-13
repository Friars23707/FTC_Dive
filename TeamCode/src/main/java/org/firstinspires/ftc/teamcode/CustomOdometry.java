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

    final double ROBOT_SPEED = 0.3;
    final double SLOW_SPEED = 0.1;
    double previousHeading = 0.0;

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
        odo.update();
        Pose2D position = odo.getPosition();
        double currentX = position.getX(DistanceUnit.INCH);
        double currentY = position.getY(DistanceUnit.INCH);

        telem.addData("cancel", "true");
        telem.addData("Target: ", "{X: %.3f, Y: %.3f}", x, y);
        telem.addData("Current: ", "{X: %.3f, Y: %.3f}", currentX, currentY);
        telem.update();

        while (!isStopRequested()) {
            telem.addData("Target: ", "{X: %.3f, Y: %.3f}", x, y);
            telem.addData("Current: ", "{X: %.3f, Y: %.3f}", currentX, currentY);
            telem.addData("In loop", "true");
            telem.update();

            currentX = position.getX(DistanceUnit.INCH);
            currentY = position.getY(DistanceUnit.INCH);

            double axial = x > currentX ? 1 : -1;
            double lateral = y > currentY ? -1 : 1;
            double yaw = 0;

            axial = Math.abs(position.getX(DistanceUnit.INCH) - x) > 10 ? axial*ROBOT_SPEED : axial*SLOW_SPEED;
            lateral = Math.abs(position.getX(DistanceUnit.INCH) - y) > 10 ? lateral*ROBOT_SPEED : lateral*SLOW_SPEED;

            double leftFrontPower  = axial + lateral + yaw;
            double rightFrontPower = axial - lateral - yaw;
            double leftBackPower   = axial - lateral + yaw;
            double rightBackPower  = axial + lateral - yaw;

            leftFrontDrive.setPower(leftFrontPower);
            rightFrontDrive.setPower(rightFrontPower);
            leftBackDrive.setPower(leftBackPower);
            rightBackDrive.setPower(rightBackPower);

            odo.update();
            position = odo.getPosition();
            boolean run1 = Math.abs(position.getX(DistanceUnit.INCH) - x) > 1;
            boolean run2 = Math.abs(position.getY(DistanceUnit.INCH) - y) > 1;
            if (!run1 && !run2) {
                leftFrontDrive.setPower(0);
                rightFrontDrive.setPower(0);
                leftBackDrive.setPower(0);
                rightBackDrive.setPower(0);
                turnTo(previousHeading);
                return;
            }
        }

    }

    public void turnTo(double heading) {
        odo.update();
        Pose2D position = odo.getPosition();
        double currentHeading = position.getHeading(AngleUnit.DEGREES);

        previousHeading = heading;

        telem.addData("cancel", "true");
        telem.addData("Target: ", "{Heading: %.3f}", heading);
        telem.addData("Current: ", "{Heading: %.3f}", currentHeading);
        telem.update();

        while (!isStopRequested()) {
            telem.addData("Target: ", "{Heading: %.3f}", heading);
            telem.addData("Current: ", "{Heading: %.3f}", currentHeading);
            telem.addData("In loop", "true");
            telem.update();

            currentHeading = radToDeg(position.getHeading(AngleUnit.DEGREES));

            double axial = 0;
            double lateral = 0;
            double yaw = heading <= currentHeading ? ROBOT_SPEED : -ROBOT_SPEED;

            double leftFrontPower  = axial + lateral + yaw;
            double rightFrontPower = axial - lateral - yaw;
            double leftBackPower   = axial - lateral + yaw;
            double rightBackPower  = axial + lateral - yaw;

            leftFrontDrive.setPower(leftFrontPower);
            rightFrontDrive.setPower(rightFrontPower);
            leftBackDrive.setPower(leftBackPower);
            rightBackDrive.setPower(rightBackPower);

            odo.update();
            position = odo.getPosition();
            boolean run3 = Math.abs(radToDeg(position.getHeading(AngleUnit.DEGREES)) - heading) > 5;
            if (!run3) {
                leftFrontDrive.setPower(0);
                rightFrontDrive.setPower(0);
                leftBackDrive.setPower(0);
                rightBackDrive.setPower(0);
                return;
            }
        }

    }

    public void moveToWithHeading(double x, double y, double heading) {
        odo.update();
        Pose2D position = odo.getPosition();
        double currentX = position.getX(DistanceUnit.INCH);
        double currentY = position.getY(DistanceUnit.INCH);
        double currentHeading = radToDeg(position.getHeading(AngleUnit.DEGREES));

        telem.addData("cancel", "true");
        telem.addData("Target: ", "{X: %.3f, Y: %.3f, Heading: %.3f}", x, y, heading);
        telem.addData("Current: ", "{X: %.3f, Y: %.3f, Heading: %.3f}", currentX, currentY, currentHeading);
        telem.update();

        while (!isStopRequested()) {
            telem.addData("Target: ", "{X: %.3f, Y: %.3f, Heading: %.3f}", x, y, heading);
            telem.addData("Current: ", "{X: %.3f, Y: %.3f, Heading: %.3f}", currentX, currentY, currentHeading);
            telem.addData("In loop", "true");
            telem.update();

            currentX = position.getX(DistanceUnit.INCH);
            currentY = position.getY(DistanceUnit.INCH);
            currentHeading = radToDeg(position.getHeading(AngleUnit.DEGREES));

            double axial = x > currentX ? 1 : -1;
            double lateral = y > currentY ? -1 : 1;
            double yaw = heading <= currentHeading ? ROBOT_SPEED : -ROBOT_SPEED;

            axial = Math.abs(position.getX(DistanceUnit.INCH) - x) > 10 ? axial*ROBOT_SPEED : axial*SLOW_SPEED;
            lateral = Math.abs(position.getX(DistanceUnit.INCH) - y) > 10 ? lateral*ROBOT_SPEED : lateral*SLOW_SPEED;

            double leftFrontPower  = axial + lateral + yaw;
            double rightFrontPower = axial - lateral - yaw;
            double leftBackPower   = axial - lateral + yaw;
            double rightBackPower  = axial + lateral - yaw;

            leftFrontDrive.setPower(leftFrontPower);
            rightFrontDrive.setPower(rightFrontPower);
            leftBackDrive.setPower(leftBackPower);
            rightBackDrive.setPower(rightBackPower);

            odo.update();
            position = odo.getPosition();
            boolean run1 = Math.abs(position.getX(DistanceUnit.INCH) - x) > 1;
            boolean run2 = Math.abs(position.getY(DistanceUnit.INCH) - y) > 1;
            boolean run3 = Math.abs(radToDeg(position.getHeading(AngleUnit.DEGREES)) - heading) > 5;
            if (!run1 && !run2 && !run3) {
                leftFrontDrive.setPower(0);
                rightFrontDrive.setPower(0);
                leftBackDrive.setPower(0);
                rightBackDrive.setPower(0);
                return;
            }
        }

    }

    public double radToDeg(double radians) {
        return  radians * (180/Math.PI);
    }

    @Override
    public void runOpMode() throws InterruptedException {}
}


