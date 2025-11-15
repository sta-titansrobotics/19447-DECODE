package com.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.teamcode.Constants;
import com.teamcode.subsystems.DriveSubsystem;
import com.teamcode.subsystems.IntakeSubsystem;
import com.teamcode.subsystems.Odometry;
import com.teamcode.util.Pose2d;
import com.teamcode.vision.AprilTagSelector;
import com.teamcode.vision.AprilTagSelector.AutoPath;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

/**
 * AprilTag-based autonomous with ball pickup and scoring.
 *
 * FLOW:
 * 1. INIT: Scan AprilTags, select path (LEFT/CENTER/RIGHT)
 * 2. START: Reset odometry, begin state machine
 * 3. DRIVE_TO_BALL: Navigate to ball pickup location
 * 4. INTAKE_BALL: Run intake while creeping forward
 * 5. DRIVE_TO_GOAL: Navigate to scoring location (varies by detected tag)
 * 6. SCORE_BALL: Execute scoring mechanism
 * 7. PARK: Drive to parking zone
 * 8. DONE: Stop all systems
 */
@Autonomous(name = "Auto_AprilTag_BallRoute", group = "Comp")
public class Auto_AprilTag_BallRoute extends LinearOpMode {

    private enum State {
        DRIVE_TO_BALL,
        INTAKE_BALL,
        DRIVE_TO_GOAL,
        SCORE_BALL,
        PARK,
        DONE
    }

    // Subsystems
    private Odometry odometry;
    private DriveSubsystem drive;
    private IntakeSubsystem intake;
    private AprilTagSelector vision;

    // State machine
    private State currentState;
    private State previousState = null; // For state entry detection
    private ElapsedTime stateTimer = new ElapsedTime();
    private ElapsedTime globalTimer = new ElapsedTime(); // Global 30s timeout
    private ElapsedTime loopTimer = new ElapsedTime();

    // Selected path from AprilTag
    private AutoPath selectedPath = AutoPath.CENTER;
    private boolean tagDetectionComplete = false;

    // Pre-allocated poses (zero-alloc)
    private Pose2d targetPose = new Pose2d();
    private Pose2d currentPose = new Pose2d();

    // Telemetry rate limiting
    private long lastTelemetryNs = 0;
    private static final long TELEMETRY_INTERVAL_NS = 100_000_000L; // 100ms = 10Hz

    // Voltage caching (update every 500ms instead of every loop)
    private double cachedVoltage = 12.0;
    private long lastVoltageReadNs = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        // ========== INITIALIZATION ==========
        telemetry.addLine("Initializing subsystems...");
        telemetry.update();

        try {
            odometry = new Odometry(hardwareMap);
            drive = new DriveSubsystem(hardwareMap, odometry);
            intake = new IntakeSubsystem(hardwareMap, Constants.INTAKE_MOTOR_NAME);
            vision = new AprilTagSelector(hardwareMap.get(WebcamName.class, Constants.WEBCAM_NAME));

            // Cache initial voltage
            if (hardwareMap.voltageSensor.iterator().hasNext()) {
                cachedVoltage = hardwareMap.voltageSensor.iterator().next().getVoltage();
            }

            // Set telemetry transmission interval (reduces Driver Station bandwidth)
            telemetry.setMsTransmissionInterval(100); // 100ms = 10Hz max

            telemetry.addLine("✓ Subsystems ready");
            telemetry.addData("Camera", vision.isCameraReady() ? "READY" : "FAILED (using default path)");
            telemetry.addData("Intake", intake.isPresent() ? "READY" : "NOT FOUND");
            telemetry.addLine("Scanning for AprilTags...");
            telemetry.update();

            // ========== INIT LOOP (AprilTag Scanning) ==========
            while (!isStarted() && !isStopRequested()) {
                vision.update();
                selectedPath = vision.getSelectedPath();

                // Track if we got a valid detection
                if (vision.hasRecentDetection()) {
                    tagDetectionComplete = true;
                }

                telemetry.addLine("=== APRILTAG SCAN ===");
                telemetry.addData("Camera", vision.isCameraReady() ? "OK" : "FAILED");
                telemetry.addData("Last Tag ID", vision.getLastSeenTagId());
                telemetry.addData("Time Since", "%.1f s", vision.getTimeSinceLastDetection());
                telemetry.addData("Selected Path", selectedPath);
                telemetry.addData("Valid Detection", vision.hasRecentDetection() ? "YES" : "NO");
                telemetry.addData("Battery", "%.1f V", cachedVoltage);

                telemetry.addLine();
                telemetry.addLine("Press START when ready");
                telemetry.update();
                idle();
            }

            if (isStopRequested()) {
                return; // Exit early, finally block will clean up
            }

        // ========== START AUTONOMOUS ==========
        telemetry.addLine("STARTING AUTONOMOUS");
        telemetry.addData("Path", selectedPath);
        telemetry.update();

        // Stop camera streaming to save resources
        vision.stopStreaming();

        // Reset odometry to starting position with encoder flush
        Pose2d startPose = new Pose2d(
            Constants.APRILTAG_AUTO_START_X,
            Constants.APRILTAG_AUTO_START_Y,
            Constants.APRILTAG_AUTO_START_HEADING
        );
        odometry.setPose(startPose);
        odometry.update();  // Flush any accumulated encoder deltas from INIT
        odometry.setPose(startPose);  // Re-apply pose after flush

        // Initialize state machine
        currentState = State.DRIVE_TO_BALL;
        previousState = null;
        stateTimer.reset();
        globalTimer.reset(); // Start global 30s timer
        loopTimer.reset();

        // ========== MAIN LOOP ==========
        while (opModeIsActive()) {
            long loopStart = System.nanoTime();

            // CRITICAL: Global 30-second timeout for FTC rule compliance
            if (globalTimer.seconds() > 29.5) {
                drive.stop();
                intake.intakeOff();
                telemetry.addLine("AUTO TIMEOUT - STOPPED AT 29.5s");
                telemetry.update();
                break;
            }

            // 1. READ SENSORS
            odometry.update();
            odometry.getPoseInto(currentPose);

            // 2. UPDATE STATE MACHINE
            updateStateMachine();

            // 3. TELEMETRY (rate-limited to 10Hz)
            long now = System.nanoTime();
            if (now - lastTelemetryNs > TELEMETRY_INTERVAL_NS) {
                updateTelemetry();

                // Track loop time
                double loopMs = (now - loopStart) / 1e6;
                telemetry.addData("Loop (ms)", "%.1f", loopMs);

                telemetry.update();
                lastTelemetryNs = now;
            }

            // 4. IDLE
            idle();
        }

        } finally {
            // ========== CLEANUP (always executes) ==========
            if (drive != null) drive.stop();
            if (intake != null) intake.intakeOff();
            if (vision != null) vision.close();

            // Safety delay for Auto→TeleOp transition
            try {
                Thread.sleep(100);
            } catch (InterruptedException e) {
                // Ignore
            }
        }
    }

    private void updateStateMachine() {
        // Detect state entry for one-time actions
        if (currentState != previousState) {
            onStateEntry(currentState);
            previousState = currentState;
        }

        switch (currentState) {
            case DRIVE_TO_BALL:
                handleDriveToBall();
                break;
            case INTAKE_BALL:
                handleIntakeBall();
                break;
            case DRIVE_TO_GOAL:
                handleDriveToGoal();
                break;
            case SCORE_BALL:
                handleScoreBall();
                break;
            case PARK:
                handlePark();
                break;
            case DONE:
                drive.stop();
                intake.intakeOff();
                break;
        }
    }

    private void handleDriveToBall() {
        // Allow AprilTag retry during first second if no valid detection in INIT
        if (!tagDetectionComplete && stateTimer.seconds() < 1.0) {
            vision.update();
            selectedPath = vision.getSelectedPath();
            if (vision.hasRecentDetection()) {
                tagDetectionComplete = true;
            }
        }

        // Set target to ball pickup location
        targetPose.x = Constants.BALL_PICKUP_X;
        targetPose.y = Constants.BALL_PICKUP_Y;
        targetPose.heading = Constants.BALL_PICKUP_HEADING;

        boolean atTarget = drive.driveToPose(
            targetPose,
            Constants.APRILTAG_POS_TOL_IN,
            Constants.APRILTAG_HEADING_TOL_RAD
        );

        if (atTarget || stateTimer.seconds() > Constants.DRIVE_TIMEOUT_S) {
            transitionTo(State.INTAKE_BALL);
        }
    }

    private void handleIntakeBall() {
        // Intake and drive commands moved to state entry
        // Just wait for duration
        if (stateTimer.seconds() > Constants.INTAKE_CREEP_DURATION_S) {
            transitionTo(State.DRIVE_TO_GOAL);
        }
    }

    private void handleDriveToGoal() {
        // Select scoring pose based on detected path
        switch (selectedPath) {
            case LEFT:
                targetPose.x = Constants.SCORE_LEFT_X;
                targetPose.y = Constants.SCORE_LEFT_Y;
                targetPose.heading = Constants.SCORE_LEFT_HEADING;
                break;
            case RIGHT:
                targetPose.x = Constants.SCORE_RIGHT_X;
                targetPose.y = Constants.SCORE_RIGHT_Y;
                targetPose.heading = Constants.SCORE_RIGHT_HEADING;
                break;
            case CENTER:
            default:
                targetPose.x = Constants.SCORE_CENTER_X;
                targetPose.y = Constants.SCORE_CENTER_Y;
                targetPose.heading = Constants.SCORE_CENTER_HEADING;
                break;
        }

        boolean atTarget = drive.driveToPose(
            targetPose,
            Constants.APRILTAG_POS_TOL_IN,
            Constants.APRILTAG_HEADING_TOL_RAD
        );

        if (atTarget || stateTimer.seconds() > Constants.DRIVE_TIMEOUT_S) {
            transitionTo(State.SCORE_BALL);
        }
    }

    private void handleScoreBall() {
        // Scoring actions moved to state entry
        // Just wait for duration
        if (stateTimer.seconds() > Constants.SCORE_TIMEOUT_S) {
            transitionTo(State.PARK);
        }
    }

    private void handlePark() {
        // Select parking location based on path
        switch (selectedPath) {
            case LEFT:
                targetPose.x = Constants.PARK_LEFT_X;
                targetPose.y = Constants.PARK_LEFT_Y;
                targetPose.heading = Constants.PARK_LEFT_HEADING;
                break;
            case RIGHT:
                targetPose.x = Constants.PARK_RIGHT_X;
                targetPose.y = Constants.PARK_RIGHT_Y;
                targetPose.heading = Constants.PARK_RIGHT_HEADING;
                break;
            case CENTER:
            default:
                targetPose.x = Constants.PARK_CENTER_X;
                targetPose.y = Constants.PARK_CENTER_Y;
                targetPose.heading = Constants.PARK_CENTER_HEADING;
                break;
        }

        boolean atTarget = drive.driveToPose(
            targetPose,
            Constants.APRILTAG_POS_TOL_IN,
            Constants.APRILTAG_HEADING_TOL_RAD
        );

        if (atTarget || stateTimer.seconds() > Constants.PARK_TIMEOUT_S) {
            drive.stop();
            transitionTo(State.DONE);
        }
    }

    private void transitionTo(State newState) {
        currentState = newState;
        stateTimer.reset();
    }

    /**
     * Called once when entering a new state.
     * Executes one-time initialization actions.
     */
    private void onStateEntry(State state) {
        switch (state) {
            case INTAKE_BALL:
                intake.intakeOn();
                drive.creepForward(Constants.INTAKE_CREEP_SPEED);
                break;

            case DRIVE_TO_GOAL:
                intake.intakeOff();
                drive.stop();
                break;

            case SCORE_BALL:
                drive.stop();
                intake.reverse();
                break;

            case PARK:
                intake.intakeOff();
                break;

            case DONE:
                drive.stop();
                intake.intakeOff();
                break;
        }
    }

    private void updateTelemetry() {
        telemetry.addLine("=== AUTONOMOUS ===");
        telemetry.addData("State", currentState);
        telemetry.addData("Path", selectedPath);
        telemetry.addData("Runtime", "%.1f / 30.0 s", globalTimer.seconds());
        telemetry.addData("State Time", "%.1f s", stateTimer.seconds());
        telemetry.addLine();

        telemetry.addLine("--- POSE ---");
        telemetry.addData("X (in)", "%.1f", currentPose.x);
        telemetry.addData("Y (in)", "%.1f", currentPose.y);
        telemetry.addData("Heading (deg)", "%.1f", Math.toDegrees(currentPose.heading));
        telemetry.addLine();

        telemetry.addLine("--- TARGET ---");
        telemetry.addData("Target X", "%.1f", targetPose.x);
        telemetry.addData("Target Y", "%.1f", targetPose.y);
        telemetry.addData("Target Hdg", "%.1f°", Math.toDegrees(targetPose.heading));
        telemetry.addLine();

        double errorDist = Math.hypot(targetPose.x - currentPose.x, targetPose.y - currentPose.y);
        telemetry.addData("Distance to Target", "%.1f in", errorDist);

        // Update cached voltage every 500ms
        long now = System.nanoTime();
        if (now - lastVoltageReadNs > 500_000_000L) {
            cachedVoltage = drive.getVoltage();
            lastVoltageReadNs = now;
        }
        telemetry.addData("Battery", "%.1f V", cachedVoltage);

        // Health warnings
        if (!drive.checkMotorHealth()) {
            telemetry.addLine("⚠️ MOTOR FAILURE DETECTED");
        }
        if (odometry.isStrafeEncoderMissing()) {
            telemetry.addLine("⚠️ 2-WHEEL ODO (strafe encoder missing)");
        }
        if (odometry.getImuFailureCount() >= 5) {
            telemetry.addLine("⚠️ IMU FAILED (encoder-only heading)");
        }
    }
}
