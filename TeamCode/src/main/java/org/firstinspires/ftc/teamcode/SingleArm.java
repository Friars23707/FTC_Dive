/*
    Our main tele op for when using the single arm bot
    Made for the FTC 2024-25 game, Into the Deep
*/

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name="SingleArm", group="TeleOps")
public class SingleArm extends LinearOpMode {

    int armPos = 0;

    // Declare OpMode members for each of the 4 motors.
    public ElapsedTime runtime = new ElapsedTime();
    public DcMotor leftFrontDrive = null;
    public DcMotor leftBackDrive = null;
    public DcMotor rightFrontDrive = null;
    public DcMotor rightBackDrive = null;
    public DcMotor leftArm = null;
    public DcMotor rightArm = null;
    public DcMotor slide = null;
    public Servo claw = null;
    public Servo wrist = null;

    @Override
    public void runOpMode() throws InterruptedException {

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "front_left");
        leftBackDrive  = hardwareMap.get(DcMotor.class, "back_left");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "front_right");
        rightBackDrive = hardwareMap.get(DcMotor.class, "back_right");

        leftArm = hardwareMap.get(DcMotor.class,"arm_left");
        rightArm = hardwareMap.get(DcMotor.class,"arm_right");
        slide = hardwareMap.get(DcMotor.class,"slide");

        claw = hardwareMap.get(Servo.class, "claw");
        wrist = hardwareMap.get(Servo.class, "wrist");


        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        // Brake when not moving
        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftArm.setDirection(DcMotor.Direction.FORWARD);
        rightArm.setDirection(DcMotor.Direction.REVERSE);
        leftArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        slide.setDirection(DcMotor.Direction.FORWARD);
        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            double max;

            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double axial   = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
            double lateral =  gamepad1.left_stick_x;
            double yaw     =  gamepad1.right_stick_x;

            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
            double leftFrontPower  = axial + lateral + yaw;
            double rightFrontPower = axial - lateral - yaw;
            double leftBackPower   = axial - lateral + yaw;
            double rightBackPower  = axial + lateral - yaw;

            int armTarget = 0;
            int slideTarget = 0;

            double clawPower = 0.5;
            double wristPower = wrist.getPosition();

            final double WRIST_FOLDED_IN   = 0.8333;
            final double WRIST_FOLDED_OUT  = 0.5;


            // Normalize the values so no wheel power exceeds 100%
            // This ensures that the robot maintains the desired motion.
            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));

            if (max > 1.0) {
                leftFrontPower  /= max;
                rightFrontPower /= max;
                leftBackPower   /= max;
                rightBackPower  /= max;
            }

            // This is test code:
            //
            // Uncomment the following code to test your motor directions.
            // Each button should make the corresponding motor run FORWARD.
            //   1) First get all the motors to take to correct positions on the robot
            //      by adjusting your Robot Configuration if necessary.
            //   2) Then make sure they run in the correct direction by modifying the
            //      the setDirection() calls above.
            // Once the correct motors move in the correct direction re-comment this code.

/*
            leftFrontPower  = gamepad1.x ? 1.0 : 0.0;  // X gamepad
            leftBackPower   = gamepad1.a ? 1.0 : 0.0;  // A gamepad
            rightFrontPower = gamepad1.y ? 1.0 : 0.0;  // Y gamepad
            rightBackPower  = gamepad1.b ? 1.0 : 0.0;  // B gamepad
*/

            // Send calculated power to wheels
            leftFrontDrive.setPower(leftFrontPower);
            rightFrontDrive.setPower(rightFrontPower);
            leftBackDrive.setPower(leftBackPower);
            rightBackDrive.setPower(rightBackPower);

            // Erlis' (stolen) arm code [its logans]
            if (gamepad2.dpad_down) { //INTAKE
                setArmPow(0.5);
                armTarget = 0;
            } else if (gamepad2.dpad_left) { //SCORE-LOW
                setArmPow(0.4);
                armTarget = 2450;
            } else if (gamepad2.dpad_up) { //SCORE-MID-LOW
                setArmPow(0.4);
                armTarget = 2800;
            } else if (gamepad2.dpad_right) { //SCORE-LOW-LOW IS TOO LOW!
                setArmPow(0.4);
                armTarget = 3000;
            } else {
                armTarget = leftArm.getTargetPosition();
            }
            if (gamepad2.left_stick_y != 0) {
                armTarget = leftArm.getCurrentPosition() - Math.round(gamepad2.left_stick_y*50);
            }


            if (gamepad2.right_bumper) {
                leftArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rightArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }

            leftArm.setTargetPosition(armTarget);
            leftArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightArm.setTargetPosition(armTarget);
            rightArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // Wrist Code
            slide.setPower(0.5);

            if (gamepad2.right_stick_y != 0) {
                slideTarget = slide.getCurrentPosition() + Math.round(gamepad2.right_stick_y*50);
            } else {
                slideTarget = slide.getTargetPosition();
            }

            slide.setTargetPosition(slideTarget);
            slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            if (gamepad2.dpad_up) {
                clawPower = 1.0;
            } else if (gamepad2.dpad_down) {
                clawPower = 0.0;
            } else {
                clawPower = 0.5;
            }

            if (gamepad2.dpad_left) {
                wristPower = (WRIST_FOLDED_IN);
            } else if (gamepad2.dpad_right) {
                wristPower = (WRIST_FOLDED_OUT);
            }

            // wristPower = gamepad2.dpad_left ? 1 : 0;
            // claw.setDirection(Servo.Direction.FORWARD);
            claw.setPosition(clawPower);
            wrist.setPosition(wristPower);

            // Show the elapsed game time and wheel power.
            telemetry.addData("Wrist", wrist.getPosition());
            telemetry.addData("Claw Input", clawPower);
            telemetry.addData("Claw", claw.getPosition());
            telemetry.addData("Slide", slideTarget+" : "+slide.getCurrentPosition());
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
            telemetry.update();
        }

    }
    public void setArmPow(double pow) {
        //Set the power for all encoders ((LEFT AND RIGHT ARM MUST MATCH OR KABOOM))
        leftArm.setPower(pow);
        rightArm.setPower(pow);
    }
}
