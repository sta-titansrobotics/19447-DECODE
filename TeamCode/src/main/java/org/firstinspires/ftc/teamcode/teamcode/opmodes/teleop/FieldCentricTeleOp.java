package com.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

import com.teamcode.subsystems.Odometry;
import com.teamcode.util.Pose2d;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/**
 * Field-Centric TeleOp with Odometry Tracking
 *
 * CONTROLS:
 *   Left Stick: Drive (forward/backward/strafe)
 *   Right Stick X: Rotate
 *   Options (PS) / Back (Xbox): Reset IMU heading
 *   Right Bumper: Slow drive mode (50% speed)
 *   Left Bumper: Reset odometry to (0, 0, 0)
 */
@TeleOp(name = "Field-Centric TeleOp", group = "TeleOp")
public class FieldCentricTeleOp extends LinearOpMode {

    // Drive motors
    private DcMotorEx frontLeft, frontRight, backLeft, backRight;

    // IMU for field-centric control
    private IMU imu;

    // Odometry for position tracking (optional but useful for debugging)
    private Odometry odometry;

    @Override
    public void runOpMode() throws InterruptedException {
        // ========== HARDWARE INITIALIZATION ==========
        initializeHardware();

        // Initialize odometry (tracks position during TeleOp)
        odometry = new Odometry(hardwareMap);

        // Set telemetry update rate
        telemetry.setMsTransmissionInterval(100); // 10Hz

        telemetry.addLine("✓ Initialized");
        telemetry.addLine("Controls:");
        telemetry.addLine("  Left Stick: Drive");
        telemetry.addLine("  Right Stick X: Rotate");
        telemetry.addLine("  Options/Back: Reset IMU");
        telemetry.addLine("  Right Bumper: Slow mode");
        telemetry.addLine("  Left Bumper: Reset odometry");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        // Reset odometry at TeleOp start
        odometry.reset(new Pose2d(0, 0, 0));

        // Main loop
        while (opModeIsActive()) {
            // ========== READ INPUTS ==========
            double y = -gamepad1.left_stick_y;   // Forward/backward (inverted)
            double x = gamepad1.left_stick_x;    // Strafe left/right
            double rx = gamepad1.right_stick_x;  // Rotation

            boolean resetIMU = gamepad1.options || gamepad1.back;  // PS Options or Xbox Back
            boolean slowMode = gamepad1.right_bumper;
            boolean resetOdometry = gamepad1.left_bumper;

            // ========== IMU RESET ==========
            if (resetIMU) {
                imu.resetYaw();
                telemetry.addLine("✓ IMU Reset");
            }

            // ========== ODOMETRY RESET ==========
            if (resetOdometry) {
                odometry.reset(new Pose2d(0, 0, 0));
                telemetry.addLine("✓ Odometry Reset");
            }

            // ========== ODOMETRY UPDATE ==========
            odometry.update();
            Pose2d pose = odometry.getPose();

            // ========== FIELD-CENTRIC DRIVE ==========
            mecanumFieldCentric(y, x, rx, slowMode);

            // ========== TELEMETRY ==========
            telemetry.addLine("=== DRIVE ===");
            telemetry.addData("Slow Mode", slowMode ? "ON (50%)" : "OFF");
            telemetry.addLine();

            telemetry.addLine("=== ODOMETRY ===");
            telemetry.addData("X (in)", "%.1f", pose.x);
            telemetry.addData("Y (in)", "%.1f", pose.y);
            telemetry.addData("Heading (deg)", "%.1f", Math.toDegrees(pose.heading));
            telemetry.addData("Vx (in/s)", "%.1f", odometry.getVx());
            telemetry.addData("Vy (in/s)", "%.1f", odometry.getVy());
            telemetry.addLine();

            telemetry.addLine("=== DIAGNOSTICS ===");
            if (odometry.isStrafeEncoderMissing()) {
                telemetry.addLine("⚠️ 2-WHEEL ODO (strafe encoder missing)");
            }
            if (odometry.getImuFailureCount() >= 5) {
                telemetry.addLine("⚠️ IMU FAILED (encoder-only heading)");
            }

            telemetry.update();
            idle();
        }
    }

    /**
     * Initialize all hardware components
     */
    private void initializeHardware() {
        // Initialize drive motors
        frontLeft  = hardwareMap.get(DcMotorEx.class, "frontLeftMotor");
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRightMotor");
        backLeft   = hardwareMap.get(DcMotorEx.class, "backLeftMotor");
        backRight  = hardwareMap.get(DcMotorEx.class, "backRightMotor");

        // Set motor directions (standard mecanum configuration)
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);

        // Set brake mode
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Run without encoders for drive (odometry uses separate encoders)
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Initialize IMU
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        imu.initialize(parameters);
    }

    /**
     * Field-centric mecanum drive with optional slow mode
     *
     * @param y Forward/backward input [-1, 1]
     * @param x Strafe left/right input [-1, 1]
     * @param rx Rotation input [-1, 1]
     * @param slowMode If true, reduces speed to 50%
     */
    private void mecanumFieldCentric(double y, double x, double rx, boolean slowMode) {
        // Get robot heading from IMU
        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        // Rotate the movement direction to be field-centric
        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        // Counteract imperfect strafing (mecanum wheels drift)
        rotX = rotX * 1.06;

        // Calculate motor powers (standard mecanum kinematics)
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1.0);
        double frontLeftPower  = (rotY + rotX + rx) / denominator;
        double backLeftPower   = (rotY - rotX + rx) / denominator;
        double frontRightPower = (rotY - rotX - rx) / denominator;
        double backRightPower  = (rotY + rotX - rx) / denominator;

        // Apply slow mode if enabled
        double speed = slowMode ? 0.5 : 1.0;

        // Set motor powers
        frontLeft.setPower(frontLeftPower * speed);
        backLeft.setPower(backLeftPower * speed);
        frontRight.setPower(frontRightPower * speed);
        backRight.setPower(backRightPower * speed);
    }
}
