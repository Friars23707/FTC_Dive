package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public class CustomOdometry extends LinearOpMode {
    private static final double MAX_SPEED = 0.6;
    private static final double MIN_SPEED = 0.15;
    private static final double HEADING_TOLERANCE = 1.0;
    private static final double POSITION_TOLERANCE = 0.3;
    private static final double RAMP_DISTANCE = 8.0;

    // PID coefficients
    private static final double kP_POS = 0.05;
    private static final double kP_HEADING = 0.03;
    private static final double kI_HEADING = 0.001;
    private static final double kD_HEADING = 0.005;

    private GoBildaPinpointDriver odo;
    private DcMotor leftFrontDrive, leftBackDrive, rightFrontDrive, rightBackDrive;
    private Telemetry telem;

    private double previousHeadingError = 0;
    private double headingIntegral = 0;

    public void initialize(HardwareMap hwm, Telemetry tm) {
        telem = tm;
        odo = hwm.get(GoBildaPinpointDriver.class, "odo");
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        odo.resetPosAndIMU();

        initializeMotors(hwm);
        resetRuntime();
    }

    private void initializeMotors(HardwareMap hwm) {
        leftFrontDrive = hwm.get(DcMotor.class, "front_left");
        leftBackDrive = hwm.get(DcMotor.class, "back_left");
        rightBackDrive = hwm.get(DcMotor.class, "back_right");
        rightFrontDrive = hwm.get(DcMotor.class, "front_right");

        DcMotor[] motors = {leftFrontDrive, leftBackDrive, rightBackDrive, rightFrontDrive};
        for (DcMotor motor : motors) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
    }

    public void moveTo(double targetX, double targetY, double targetHeading) {
        // Reset PID terms at start
        headingIntegral = 0;
        previousHeadingError = 0;

        Pose2D currentPose;
        double lastUpdateTime = System.nanoTime();

        while (!isStopRequested()) {
            odo.update();
            currentPose = odo.getPosition();
            double currentTime = System.nanoTime();
            double dt = (currentTime - lastUpdateTime) / 1e9;
            lastUpdateTime = currentTime;

            // Calculate position errors
            double xError = targetX - currentPose.getX(DistanceUnit.INCH);
            double yError = targetY - currentPose.getY(DistanceUnit.INCH);

            // Calculate heading error with angle wrapping
            double headingError = normalizeAngle(targetHeading - currentPose.getHeading(AngleUnit.DEGREES));
            // Calculate field-centric movement vectors
            double robotHeading = currentPose.getHeading(AngleUnit.RADIANS);
            double rotatedX = xError * Math.cos(robotHeading) + yError * Math.sin(robotHeading);
            double rotatedY = -xError * Math.sin(robotHeading) + yError * Math.cos(robotHeading);

            // Calculate power components with PID
            double axialPower = calculatePositionPower(rotatedX);
            double lateralPower = calculatePositionPower(rotatedY);
            double yawPower = calculateHeadingPower(headingError, dt);

            // Apply motor powers with normalization
            setMotorPowers(axialPower, lateralPower, yawPower);

            // Update telemetry
            updateTelemetry(currentPose, targetX, targetY, targetHeading);

            // Check exit conditions
            if (Math.hypot(xError, yError) < POSITION_TOLERANCE &&
                    Math.abs(headingError) < HEADING_TOLERANCE) {
                stopMotors();
                return;
            }
        }
    }

    private double calculatePositionPower(double error) {
        double power = error * kP_POS;
        double speedLimit = Math.abs(error) > RAMP_DISTANCE ? MAX_SPEED :
                MIN_SPEED + (MAX_SPEED - MIN_SPEED) * (Math.abs(error) / RAMP_DISTANCE);
        return Math.max(-speedLimit, Math.min(speedLimit, power));
    }

    private double calculateHeadingPower(double error, double dt) {
        // PID controller for heading
        headingIntegral += error * dt;
        double derivative = (error - previousHeadingError) / dt;
        previousHeadingError = error;

        return error * kP_HEADING +
                headingIntegral * kI_HEADING +
                derivative * kD_HEADING;
    }

    private void setMotorPowers(double axial, double lateral, double yaw) {
        // Mecanum wheel calculations with power clamping
        double[] powers = {
                axial + lateral + yaw,
                axial - lateral - yaw,
                axial - lateral + yaw,
                axial + lateral - yaw
        };

        // Normalize powers while maintaining ratios
        double maxPower = Math.max(1.0,
                Math.max(Math.max(Math.abs(powers[0]), Math.abs(powers[1])),
                        Math.max(Math.abs(powers[2]), Math.abs(powers[3])))
        );

        for (int i = 0; i < powers.length; i++) {
            powers[i] /= maxPower;
            powers[i] = Math.max(-MAX_SPEED, Math.min(MAX_SPEED, powers[i]));
        }

        leftFrontDrive.setPower(powers[0]);
        rightFrontDrive.setPower(powers[1]);
        leftBackDrive.setPower(powers[2]);
        rightBackDrive.setPower(powers[3]);
    }

    private double normalizeAngle(double degrees) {
        while (degrees > 180) degrees -= 360;
        while (degrees <= -180) degrees += 360;
        return degrees;
    }

    private void updateTelemetry(Pose2D currentPose, double tX, double tY, double tH) {
        telem.addData("Target", "X: %.2f, Y: %.2f, H: %.1f", tX, tY, tH);
        telem.addData("Current", "X: %.2f, Y: %.2f, H: %.1f",
                currentPose.getX(DistanceUnit.INCH),
                currentPose.getY(DistanceUnit.INCH),
                radToDeg(currentPose.getHeading(AngleUnit.DEGREES))
        );
        telem.update();
    }

    private void stopMotors() {
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
    }

    private double radToDeg(double radians) {
        return Math.toDegrees(radians);
    }

    @Override
    public void runOpMode() throws InterruptedException {}
}