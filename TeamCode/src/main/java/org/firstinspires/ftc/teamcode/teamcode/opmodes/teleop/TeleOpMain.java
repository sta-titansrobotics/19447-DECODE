package com.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.teamcode.Constants;
import com.teamcode.subsystems.DriveSubsystem;
import com.teamcode.subsystems.FeederSubsystem;
import com.teamcode.subsystems.IntakeSubsystem;
import com.teamcode.subsystems.Odometry;
import com.teamcode.subsystems.ShooterSubsystem;
import com.teamcode.util.Pose2d;

@TeleOp(name = "TeleOp Main", group = "Main")
public class TeleOpMain extends LinearOpMode {

    // Subsystems
    private DriveSubsystem drive;
    private Odometry odometry;
    private ShooterSubsystem shooter;
    private FeederSubsystem feeder;
    private IntakeSubsystem intake;

    // Button edge detection
    private boolean lastDpadUp = false;
    private boolean lastX = false;
    private boolean lastY = false;
    private boolean lastB = false;

    // Telemetry rate limiting
    private long lastTelemetryNs = 0;
    private static final long TELEMETRY_INTERVAL_NS = 100_000_000L; // 10Hz

    @Override
    public void runOpMode() throws InterruptedException {
        // ========== INITIALIZATION ==========
        telemetry.addLine("Initializing TeleOp...");
        telemetry.update();

        odometry = new Odometry(hardwareMap);
        drive = new DriveSubsystem(hardwareMap, odometry);
        shooter = new ShooterSubsystem(hardwareMap);
        feeder = new FeederSubsystem(hardwareMap);
        intake = new IntakeSubsystem(hardwareMap, Constants.INTAKE_MOTOR_NAME);

        // Set shooter to medium idle speed
        shooter.setSpeedMode(ShooterSubsystem.SpeedMode.MEDIUM);

        telemetry.setMsTransmissionInterval(100); // SDK-level rate limit
        telemetry.addLine("✓ Ready");
        telemetry.addLine("Controls:");
        telemetry.addLine("  GP1: Drive (left stick + right stick)");
        telemetry.addLine("  GP2 LT: Intake");
        telemetry.addLine("  GP2 RT: Shoot");
        telemetry.addLine("  GP2 X/Y/B: Shooter speed");
        telemetry.addLine("  GP2 D-up: Outtake toggle");
        telemetry.update();

        waitForStart();

        // Reset odometry at start (optional - could preserve from auto)
        odometry.reset(new Pose2d(0, 0, 0));

        // ========== MAIN LOOP ==========
        while (opModeIsActive()) {
            // 1. READ INPUTS
            readInputs();

            // 2. UPDATE SUBSYSTEMS
            odometry.update();

            // Drive (gamepad1)
            double forward = -gamepad1.left_stick_y;  // Inverted (up = positive)
            double strafe = gamepad1.left_stick_x;
            double turn = gamepad1.right_stick_x;
            double speedMultiplier = gamepad1.right_bumper
                ? Constants.TELEOP_DRIVE_SPEED_PRECISION
                : Constants.TELEOP_DRIVE_SPEED_NORMAL;
            drive.teleopDrive(forward, strafe, turn, speedMultiplier);

            // Intake (gamepad2)
            intake.update(gamepad2.left_trigger);

            // Shooter + Feeder (gamepad2)
            boolean shootCommand = gamepad2.right_trigger > Constants.TRIGGER_THRESHOLD;
            shooter.setShootCommand(shootCommand);
            feeder.setFeedCommand(shootCommand);

            shooter.update();
            feeder.update();

            // 3. TELEMETRY (rate-limited)
            long now = System.nanoTime();
            if (now - lastTelemetryNs > TELEMETRY_INTERVAL_NS) {
                updateTelemetry();
                telemetry.update();
                lastTelemetryNs = now;
            }

            idle();
        }

        // ========== CLEANUP ==========
        drive.stop();
        shooter.stop();
        feeder.stop();
        intake.intakeOff();
    }

    /**
     * Read gamepad inputs and handle button edge detection.
     */
    private void readInputs() {
        // Outtake toggle (dpad_up rising edge)
        boolean dpadUpNow = gamepad2.dpad_up;
        if (dpadUpNow && !lastDpadUp) {
            intake.toggleOuttakeMode();
        }
        lastDpadUp = dpadUpNow;

        // Shooter speed mode changes (X/Y/B rising edges)
        if (gamepad2.x && !lastX) {
            shooter.setSpeedMode(ShooterSubsystem.SpeedMode.LOW);
        }
        if (gamepad2.y && !lastY) {
            shooter.setSpeedMode(ShooterSubsystem.SpeedMode.MEDIUM);
        }
        if (gamepad2.b && !lastB) {
            shooter.setSpeedMode(ShooterSubsystem.SpeedMode.MAX);
        }
        lastX = gamepad2.x;
        lastY = gamepad2.y;
        lastB = gamepad2.b;
    }

    /**
     * Update telemetry with subsystem status.
     */
    private void updateTelemetry() {
        Pose2d pose = odometry.getPose();

        telemetry.addLine("=== DRIVE ===");
        telemetry.addData("Speed Mode", gamepad1.right_bumper ? "PRECISION" : "NORMAL");
        telemetry.addData("Pose", "X=%.1f Y=%.1f H=%.0f°",
            pose.x, pose.y, Math.toDegrees(pose.heading));
        telemetry.addLine();

        telemetry.addLine("=== SHOOTER ===");
        telemetry.addData("Mode", shooter.getSpeedMode());
        telemetry.addData("Target RPM", "%.0f", shooter.getTargetRPM());
        telemetry.addData("Current RPM", "%.0f", shooter.getVelocityRPM());
        telemetry.addData("At Speed", shooter.isAtSpeed() ? "YES" : "NO");
        telemetry.addData("Shooting", shooter.isShootCommandActive() ? "ACTIVE" : "idle");
        telemetry.addLine();

        telemetry.addLine("=== INTAKE/FEEDER ===");
        telemetry.addData("Intake Power", "%.2f", intake.getPower());
        telemetry.addData("Outtake Mode", intake.isOuttakeModeActive() ? "ON" : "off");
        telemetry.addData("Feeder Power", "%.2f", feeder.getPower());
        telemetry.addLine();

        // Health warnings
        if (!drive.checkMotorHealth()) {
            telemetry.addLine("⚠️ DRIVE MOTOR FAILURE");
        }
        if (odometry.isStrafeEncoderMissing()) {
            telemetry.addLine("⚠️ 2-WHEEL ODO");
        }
        if (odometry.getImuFailureCount() >= 5) {
            telemetry.addLine("⚠️ IMU FAILED");
        }
    }
}
