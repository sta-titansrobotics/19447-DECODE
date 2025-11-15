package com.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.teamcode.Constants;
import com.teamcode.subsystems.DriveSubsystem;
import com.teamcode.subsystems.Odometry;
import com.teamcode.util.Pose2d;

@Autonomous(name = "AutoRightBasic", group = "Auto")
public class AutoRightBasic extends LinearOpMode {

    private enum State {
        DRIVE_TO_SCORE,
        SCORE_PRELOAD,
        DRIVE_TO_PARK,
        IDLE
    }

    private Odometry odo;
    private DriveSubsystem drive;
    private State state;
    private ElapsedTime stateTimer;
    private ElapsedTime runtime;

    // Telemetry rate limiting
    private long lastTelemetryNs = 0;
    private static final long TELEMETRY_INTERVAL_NS = 100_000_000L; // 100ms = 10Hz

    // Cached target poses to avoid per-loop allocations
    private Pose2d targetScore;
    private Pose2d targetPark;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize subsystems
        odo = new Odometry(hardwareMap);
        drive = new DriveSubsystem(hardwareMap, odo);
        stateTimer = new ElapsedTime();
        runtime = new ElapsedTime();

        // Pre-allocate target poses (avoids GC during loop)
        targetScore = new Pose2d(
            Constants.AUTO_SCORE_X,
            Constants.AUTO_SCORE_Y,
            Constants.AUTO_SCORE_HEADING_RAD
        );
        targetPark = new Pose2d(
            Constants.AUTO_PARK_X,
            Constants.AUTO_PARK_Y,
            Constants.AUTO_PARK_HEADING_RAD
        );

        // Set starting pose and flush encoder deltas
        Pose2d startPose = new Pose2d(
            Constants.AUTO_START_X,
            Constants.AUTO_START_Y,
            Constants.AUTO_START_HEADING_RAD
        );
        odo.setPose(startPose);
        odo.update();  // Flush any accumulated encoder deltas from INIT
        odo.setPose(startPose);  // Re-apply pose after flush

        // Set telemetry transmission interval (reduces Driver Station bandwidth)
        telemetry.setMsTransmissionInterval(100); // 100ms = 10Hz max

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Start Pose", "X=%.1f Y=%.1f H=%.1f°",
            startPose.x, startPose.y, Math.toDegrees(startPose.heading));
        telemetry.update();

        waitForStart();
        runtime.reset();

        // Begin autonomous
        state = State.DRIVE_TO_SCORE;
        stateTimer.reset();

        while (opModeIsActive()) {
            // CRITICAL: Global 30-second timeout for FTC rule compliance
            if (runtime.seconds() > 29.5) {
                drive.stop();
                telemetry.addLine("AUTO TIMEOUT - STOPPED AT 29.5s");
                telemetry.update();
                break;
            }

            // 1) Update odometry
            odo.update();

            // 2) State machine
            switch (state) {
                case DRIVE_TO_SCORE:
                    handleDriveToScore();
                    break;
                case SCORE_PRELOAD:
                    handleScorePreload();
                    break;
                case DRIVE_TO_PARK:
                    handleDriveToPark();
                    break;
                case IDLE:
                    drive.stop();
                    break;
            }

            // 3) Telemetry (rate-limited to 10Hz to prevent I2C congestion)
            long now = System.nanoTime();
            if (now - lastTelemetryNs > TELEMETRY_INTERVAL_NS) {
                updateTelemetry();
                telemetry.update();
                lastTelemetryNs = now;
            }

            idle();
        }

        // Safety stop for Auto→TeleOp transition
        drive.stop();
        sleep(100);  // Ensure stop command is sent before transition
    }

    private void handleDriveToScore() {
        if (driveToTarget(targetScore) || stateTimer.seconds() > Constants.AUTO_STEP_TIMEOUT_S) {
            // Reached target or timeout
            drive.stop();
            transitionTo(State.SCORE_PRELOAD);
        }
    }

    private void handleScorePreload() {
        // Simulate scoring action (e.g., actuator movement)
        // In real implementation, control servos/motors here
        drive.stop();

        if (stateTimer.seconds() > Constants.AUTO_SCORE_DURATION_S) {
            transitionTo(State.DRIVE_TO_PARK);
        }
    }

    private void handleDriveToPark() {
        if (driveToTarget(targetPark) || stateTimer.seconds() > Constants.AUTO_STEP_TIMEOUT_S) {
            drive.stop();
            transitionTo(State.IDLE);
        }
    }

    /**
     * Drive to target pose using DriveSubsystem's built-in controller.
     * @return true if within tolerance, false otherwise
     */
    private boolean driveToTarget(Pose2d target) {
        // DriveSubsystem.driveToPose() handles all the P-control logic
        return drive.driveToPose(target, Constants.AUTO_POS_TOLERANCE_IN, Constants.AUTO_ANGLE_TOLERANCE_RAD);
    }

    private void transitionTo(State newState) {
        state = newState;
        stateTimer.reset();
    }

    private void updateTelemetry() {
        Pose2d pose = odo.getPose();
        telemetry.addData("State", state);
        telemetry.addData("Runtime", "%.1f / 30.0 s", runtime.seconds());
        telemetry.addData("State Time", "%.2f s", stateTimer.seconds());
        telemetry.addData("Pose", "X=%.1f Y=%.1f H=%.1f°",
            pose.x, pose.y, Math.toDegrees(pose.heading));
        telemetry.addData("Velocity", "Vx=%.1f Vy=%.1f Ω=%.2f",
            odo.getVx(), odo.getVy(), Math.toDegrees(odo.getOmega()));

        // Health warnings
        if (!drive.checkMotorHealth()) {
            telemetry.addLine("⚠️ MOTOR FAILURE DETECTED");
        }
        if (odo.isStrafeEncoderMissing()) {
            telemetry.addLine("⚠️ 2-WHEEL ODO (strafe encoder missing)");
        }
        if (odo.getImuFailureCount() >= 5) {
            telemetry.addLine("⚠️ IMU FAILED (encoder-only heading)");
        }
    }
}
