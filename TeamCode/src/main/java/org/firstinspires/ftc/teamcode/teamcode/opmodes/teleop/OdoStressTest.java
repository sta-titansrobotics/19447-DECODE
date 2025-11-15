package com.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.teamcode.subsystems.Odometry;
import com.teamcode.util.Pose2d;

/**
 * STRESS TEST OpMode for odometry subsystem.
 * Tests encoder deadband, dt stability, and allocation performance under various conditions.
 *
 * CONTROLS:
 *   A - Mode: Stationary drift test (check encoder deadband)
 *   B - Mode: Normal driving (manual control)
 *   X - Mode: High-frequency telemetry spam (dt stability test)
 *   Y - Reset statistics
 *   DPad Up/Down - Adjust telemetry update rate
 */
@TeleOp(name = "OdoStressTest", group = "Test")
public class OdoStressTest extends LinearOpMode {

    private enum TestMode {
        STATIONARY,    // Robot sitting still → verify no drift from encoder noise
        NORMAL,        // Normal operation
        SPAM_TELEMETRY // High telemetry rate → check if dt spikes
    }

    private TestMode mode = TestMode.NORMAL;

    // Statistics
    private double dtSum = 0.0;
    private double dtMin = Double.MAX_VALUE;
    private double dtMax = 0.0;
    private int loopCount = 0;

    // Zero-allocation pose read
    private Pose2d currentPose = new Pose2d();
    private Pose2d startPose = new Pose2d();

    // Telemetry rate control
    private int telemetryRateMs = 100;
    private long lastTelemetryNs = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        Odometry odo = new Odometry(hardwareMap);

        VoltageSensor vs = hardwareMap.voltageSensor.iterator().hasNext()
                ? hardwareMap.voltageSensor.iterator().next()
                : null;

        telemetry.addLine("ODOMETRY STRESS TEST");
        telemetry.addLine("A=Stationary | B=Normal | X=Spam | Y=Reset Stats");
        telemetry.addLine("DPad Up/Down = Telemetry rate");
        telemetry.update();

        waitForStart();
        resetStats();

        long lastNs = System.nanoTime();
        boolean lastA = false, lastB = false, lastX = false, lastY = false;
        boolean lastDpadUp = false, lastDpadDown = false;

        while (opModeIsActive()) {
            long now = System.nanoTime();
            double dt = (now - lastNs) / 1e9;
            lastNs = now;

            // Track dt statistics
            if (dt > 0) {
                dtSum += dt;
                dtMin = Math.min(dtMin, dt);
                dtMax = Math.max(dtMax, dt);
                loopCount++;
            }

            // Mode selection (rising edge detection)
            if (gamepad1.a && !lastA) {
                mode = TestMode.STATIONARY;
                odo.getPoseInto(startPose); // Record starting pose
                resetStats();
            }
            if (gamepad1.b && !lastB) {
                mode = TestMode.NORMAL;
                resetStats();
            }
            if (gamepad1.x && !lastX) {
                mode = TestMode.SPAM_TELEMETRY;
                resetStats();
            }
            if (gamepad1.y && !lastY) {
                resetStats();
            }

            lastA = gamepad1.a;
            lastB = gamepad1.b;
            lastX = gamepad1.x;
            lastY = gamepad1.y;

            // Telemetry rate control
            if (gamepad1.dpad_up && !lastDpadUp) {
                telemetryRateMs = Math.max(20, telemetryRateMs - 20);
            }
            if (gamepad1.dpad_down && !lastDpadDown) {
                telemetryRateMs = Math.min(500, telemetryRateMs + 20);
            }
            lastDpadUp = gamepad1.dpad_up;
            lastDpadDown = gamepad1.dpad_down;

            // CRITICAL: Update odometry every loop
            odo.update();

            // Read pose with zero allocation
            odo.getPoseInto(currentPose);

            // Telemetry logic based on mode and rate limit
            long telemetryDeltaNs = now - lastTelemetryNs;
            boolean shouldUpdateTelemetry = false;

            switch (mode) {
                case STATIONARY:
                    shouldUpdateTelemetry = telemetryDeltaNs >= telemetryRateMs * 1_000_000L;
                    break;
                case NORMAL:
                    shouldUpdateTelemetry = telemetryDeltaNs >= telemetryRateMs * 1_000_000L;
                    break;
                case SPAM_TELEMETRY:
                    shouldUpdateTelemetry = true; // Every loop
                    break;
            }

            if (shouldUpdateTelemetry) {
                lastTelemetryNs = now;
                updateTelemetryDisplay(odo, vs, dt);
            }

            idle();
        }
    }

    private void updateTelemetryDisplay(Odometry odo, VoltageSensor vs, double dt) {
        double dtAvg = loopCount > 0 ? dtSum / loopCount : 0.0;

        telemetry.addLine("=== STRESS TEST ===");
        telemetry.addData("Mode", mode);
        telemetry.addData("Telemetry Rate", "%d ms", telemetryRateMs);
        telemetry.addLine();

        telemetry.addLine("--- LOOP TIMING ---");
        telemetry.addData("dt (ms)", "cur=%.1f | avg=%.1f | min=%.1f | max=%.1f",
                dt * 1000.0, dtAvg * 1000.0, dtMin * 1000.0, dtMax * 1000.0);
        telemetry.addData("Loop count", loopCount);
        telemetry.addLine();

        telemetry.addLine("--- ODOMETRY ---");
        telemetry.addData("x (in)", "%.3f", currentPose.x);
        telemetry.addData("y (in)", "%.3f", currentPose.y);
        telemetry.addData("hdg (deg)", "%.2f", Math.toDegrees(currentPose.heading));
        telemetry.addData("vx (in/s)", "%.2f", odo.getVx());
        telemetry.addData("vy (in/s)", "%.2f", odo.getVy());
        telemetry.addData("ω (deg/s)", "%.1f", Math.toDegrees(odo.getOmega()));

        if (mode == TestMode.STATIONARY) {
            double driftX = currentPose.x - startPose.x;
            double driftY = currentPose.y - startPose.y;
            double driftDist = Math.hypot(driftX, driftY);
            double driftHdg = Math.toDegrees(currentPose.heading - startPose.heading);

            telemetry.addLine();
            telemetry.addLine("--- DRIFT (from start) ---");
            telemetry.addData("Δx (in)", "%.4f", driftX);
            telemetry.addData("Δy (in)", "%.4f", driftY);
            telemetry.addData("Distance", "%.4f in", driftDist);
            telemetry.addData("Δhdg (deg)", "%.3f", driftHdg);
        }

        telemetry.addLine();
        if (vs != null) {
            telemetry.addData("Battery (V)", "%.2f", vs.getVoltage());
        }

        telemetry.update();
    }

    private void resetStats() {
        dtSum = 0.0;
        dtMin = Double.MAX_VALUE;
        dtMax = 0.0;
        loopCount = 0;
    }
}
