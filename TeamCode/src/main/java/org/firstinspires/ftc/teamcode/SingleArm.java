/*
    Our main tele op for when using the single arm bot
    Made for the FTC 2024-25 game, Into the Deep
*/

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

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


@TeleOp(name="SingleArm", group="TeleOps")
public class SingleArm extends OpMode {

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

    public ColorRangeSensor sampleSensor = null;

    public Servo leftLight = null;
    public Servo rightLight = null;

    int armTarget = 0;
    int slideTarget = 0;

    double clawPower = 0.5;
    double wristPos = 0.5;

    boolean redAlliance = false;

    @Override
    public void init() {
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

        sampleSensor = hardwareMap.get(ColorRangeSensor.class, "sample_sens");

        leftLight = hardwareMap.get(Servo.class, "left_tower");
        rightLight = hardwareMap.get(Servo.class, "right_tower");

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        // Brake when not moving
        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftArm.setDirection(DcMotor.Direction.REVERSE);
        rightArm.setDirection(DcMotor.Direction.FORWARD);
        leftArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        slide.setDirection(DcMotor.Direction.FORWARD);
        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


    }

    @Override
    public void init_loop() {
        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.addData("Alliance", redAlliance ? "Red" : "Blue");
        telemetry.update();

        if (gamepad1.x) {
            redAlliance = false;
        } else if (gamepad1.b) {
            redAlliance = true;
        }
    }

    @Override
    public  void start() {
        runtime.reset();
    }

    @Override
    public void loop() {
        double max;

        // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
        double axial   = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
        double lateral =  gamepad1.left_stick_x;
        double yaw     =  gamepad1.right_stick_x/3;

        // Combine the joystick requests for each axis-motion to determine each wheel's power.
        // Set up a variable for each drive wheel to save the power level for telemetry.
        double leftFrontPower  = axial + lateral + yaw;
        double rightFrontPower = axial - lateral - yaw;
        double leftBackPower   = axial - lateral + yaw;
        double rightBackPower  = axial + lateral - yaw;

        final double WRIST_FOLDED_IN   = 0.12;
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
        if (gamepad2.dpad_down) { //FLOOR INTAKE
            setArmPow(0.5);
            armTarget = 0;
        } else if (gamepad2.dpad_left) { //LOW BASKET
            setArmPow(0.4);
            armTarget = 1000;
        } else if (gamepad2.dpad_up) { //HIGH BASKET
            setArmPow(0.4);
            armTarget = 1500;
        } else if (gamepad2.dpad_right) { //HANG
            setArmPow(0.8);
            armTarget = 4600;
        } else {
            armTarget = leftArm.getTargetPosition();
        }

        if (gamepad2.left_stick_y != 0) {
            setArmPow(0.5);
            armTarget = leftArm.getCurrentPosition() - Math.round(gamepad2.left_stick_y*50);
        }

        //armTarget = Math.min(armTarget, 00);

        if (gamepad2.right_bumper) {
            leftArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }

        leftArm.setTargetPosition(armTarget);
        leftArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightArm.setTargetPosition(armTarget);
        rightArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Slide Code
        slide.setPower(0.5);

        if (gamepad2.right_stick_y != 0) {
            slideTarget = slide.getCurrentPosition() + Math.round(gamepad2.right_stick_y*50);
        } else {
            slideTarget = slide.getCurrentPosition();
        }

        slide.setTargetPosition(slideTarget);
        slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        if (gamepad2.y) {
            clawPower = 1.0;
        } else if (gamepad2.a) {
            clawPower = 0.0;
        } else {
            clawPower = 0.5;
        }

        if (gamepad2.x) {
            wristPos = (WRIST_FOLDED_IN);
        } else if (gamepad2.b) {
            wristPos = (WRIST_FOLDED_OUT);
        }

        // wristPower = gamepad2.dpad_left ? 1 : 0;
        // claw.setDirection(Servo.Direction.FORWARD);

        int sampleRed = sampleSensor.red();
        int sampleGreen = sampleSensor.green();
        int sampleBlue = sampleSensor.blue();
        double lightColor1;

        if (sampleRed >= sampleBlue && sampleRed >= sampleGreen) {
            lightColor1 = 0.279;
        } else if (sampleBlue >= sampleRed && sampleBlue >= sampleGreen) {
            lightColor1 = 0.611;
        } else {
            lightColor1 = 0.388;
        }

        leftLight.setPosition(lightColor1);

        double sampleDistance = sampleSensor.getDistance(DistanceUnit.INCH);
        double lightColor2 = smoothMap(sampleDistance);

        rightLight.setPosition(lightColor2);

        if (lightColor1 == 0.279 && !redAlliance) {
            clawPower = 1.0;
        } else if (lightColor1 == 0.611 && redAlliance) {
            clawPower = 1.0;
        }

        claw.setPosition(clawPower);
        wrist.setPosition(wristPos);

        // Show the elapsed game time and wheel power.
        telemetry.addData("Wrist", wrist.getPosition());
        telemetry.addData("Claw Input", clawPower);
        telemetry.addData("Claw", claw.getPosition());
        telemetry.addData("Slide", slideTarget+" : "+slide.getCurrentPosition());
        telemetry.addData("Arm", armTarget+" : "+leftArm.getCurrentPosition());
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
        telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
        telemetry.addData("Sample Color", lightColor1);
        telemetry.addData("Sample Distance", sampleDistance);
        telemetry.addData("Sample Distance Clr", lightColor2);
        telemetry.addData("Alliance", redAlliance ? "Red" : "Blue");
        telemetry.update();
    }

    public void setArmPow(double pow) {
        //Set the power for all encoders ((LEFT AND RIGHT ARM MUST MATCH OR KABOOM))

        leftArm.setPower(pow);
        rightArm.setPower(pow);
    }

    //SMOOTH MAP LINE CREATION
    double x1 = 0.2, y1 = 0.5;
    double x2 = 1.4, y2 = 0.279;

    // Linear interpolation formula
    double slope = (y2 - y1) / (x2 - x1);
    double intercept = y1 - slope * x1;

    public double smoothMap(double input) {
        // Apply linear interpolation to get the result
        double output = slope * input + intercept;

        //Apply limits to the color range
        output = Math.min(output, 0.6);
        output = Math.max(output, 0.279);

        return output;
    }

}
