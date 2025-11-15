package com.teamcode.subsystems;

import static com.teamcode.Constants.*;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.teamcode.hardware.Encoder;
import com.teamcode.util.Angle;
import com.teamcode.util.Pose2d;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.List;

public class Odometry {
    // Noise suppression: ignore encoder deltas smaller than this (stationary jitter)
    private static final int ENCODER_DEADBAND_TICKS = 2;

    private final HardwareMap hw;

    private final Encoder leftEnc;
    private final Encoder rightEnc;
    private final Encoder strafeEnc; // may be null

    private final IMU imu;
    private double imuHeadingRad = 0.0;
    private int imuFailureCount = 0; // Count consecutive failures before giving up
    private static final int IMU_MAX_FAILURES = 5; // Allow recovery after transient glitches

    private Pose2d pose = new Pose2d();
    private double vx = 0.0, vy = 0.0, omega = 0.0; // robot-centric velocities

    private final ElapsedTime loopTimer = new ElapsedTime();
    private double lastHeadingRad = 0.0;

    private boolean strafeEncoderMissing = false; // Track if strafe encoder is disabled

    private static double ticksToInches(int ticks) {
        double wheelCircum = Math.PI * WHEEL_DIAMETER_IN;
        return (ticks / TICKS_PER_REV) * (wheelCircum * GEAR_RATIO);
    }

    public Odometry(HardwareMap hw) {
        this.hw = hw;

        // Sanity checks to prevent NaN/Inf during math
        if (TRACK_WIDTH_IN <= 0) {
            throw new IllegalArgumentException("TRACK_WIDTH_IN must be > 0");
        }
        if (TICKS_PER_REV <= 0) {
            throw new IllegalArgumentException("TICKS_PER_REV must be > 0");
        }

        // Bulk caching: minimize read latency
        for (LynxModule module : hw.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        leftEnc   = new Encoder(hw, ENC_LEFT,   LEFT_DIR);
        rightEnc  = new Encoder(hw, ENC_RIGHT,  RIGHT_DIR);
        strafeEnc = new Encoder(hw, ENC_STRAFE, STRAFE_DIR);

        // Check if strafe encoder is missing (two-wheel odometry mode)
        strafeEncoderMissing = (strafeEnc == null || !strafeEnc.isPresent());

        if (USE_IMU) {
            imu = hw.get(IMU.class, IMU_NAME);
            IMU.Parameters params = new IMU.Parameters(new RevHubOrientationOnRobot(
                    RevHubOrientationOnRobot.LogoFacingDirection.UP,
                    RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
            imu.initialize(params);
            imu.resetYaw();
            imuHeadingRad = 0.0;
        } else {
            imu = null;
        }

        reset(new Pose2d(0,0,0));
        loopTimer.reset();
    }

    public void reset(Pose2d start) {
        if (leftEnc.isPresent())  leftEnc.reset();
        if (rightEnc.isPresent()) rightEnc.reset();
        if (strafeEnc != null && strafeEnc.isPresent()) strafeEnc.reset();
        pose = start.copy();
        lastHeadingRad = pose.heading;
        if (imu != null) {
            imu.resetYaw();
            imuHeadingRad = 0.0;
        }
        loopTimer.reset();
        vx = vy = omega = 0.0;
    }

    public void setPose(Pose2d p) { this.pose = p.copy(); this.lastHeadingRad = pose.heading; }

    /**
     * Returns a COPY of current pose (allocates new object).
     * For high-frequency access, prefer getPoseInto() to avoid GC.
     */
    public Pose2d getPose() { return pose.copy(); }

    /**
     * Zero-allocation pose getter. Writes current pose into provided object.
     * @param out Pose2d to write into (modified in-place)
     */
    public void getPoseInto(Pose2d out) {
        out.x = pose.x;
        out.y = pose.y;
        out.heading = pose.heading;
    }

    /** @return Forward velocity in robot frame (inches/s) */
    public double getVx() { return vx; }

    /** @return Lateral velocity in robot frame (inches/s, left+) */
    public double getVy() { return vy; }

    /** @return Angular velocity (rad/s, CCW+) */
    public double getOmega() { return omega; }

    /** @return true if using two-wheel odometry (strafe encoder missing) */
    public boolean isStrafeEncoderMissing() { return strafeEncoderMissing; }

    /** @return IMU failure count (for diagnostics) */
    public int getImuFailureCount() { return imuFailureCount; }

    // Main update; call every loop
    public void update() {
        double dt = loopTimer.seconds();
        loopTimer.reset();
        if (dt <= 0) {
            // Invalid dt → zero velocities and skip update
            vx = vy = omega = 0.0;
            return;
        }
        if (dt > MAX_DT_S) dt = MAX_DT_S;

        // --- readSensors()
        int dL_ticks = leftEnc.getDeltaTicks();
        int dR_ticks = rightEnc.getDeltaTicks();
        int dS_ticks = (strafeEnc != null && strafeEnc.isPresent()) ? strafeEnc.getDeltaTicks() : 0;

        // Apply deadband to suppress encoder noise when stationary
        if (Math.abs(dL_ticks) <= ENCODER_DEADBAND_TICKS) dL_ticks = 0;
        if (Math.abs(dR_ticks) <= ENCODER_DEADBAND_TICKS) dR_ticks = 0;
        if (Math.abs(dS_ticks) <= ENCODER_DEADBAND_TICKS) dS_ticks = 0;

        double dL = ticksToInches(dL_ticks);
        double dR = ticksToInches(dR_ticks);
        double dS = ticksToInches(dS_ticks);

        // --- compute() three-wheel kinematics
        double dTheta_enc = (dR - dL) / TRACK_WIDTH_IN;
        double headingEnc = Angle.norm(lastHeadingRad + dTheta_enc);

        // Lateral correction for wheel offset
        double lateral = dS - dTheta_enc * LATERAL_WHEEL_OFFSET_IN;
        double forward = (dL + dR) * 0.5;

        // Exact arc correction for rotation (handles large angles correctly)
        double dX_robot, dY_robot;
        if (Math.abs(dTheta_enc) < 1e-6) {
            // Straight line motion (avoid division by zero)
            dX_robot = forward;
            dY_robot = lateral;
        } else {
            // Arc correction for rotation using sinc approximation
            double sinTheta = Math.sin(dTheta_enc);
            double cosTheta = Math.cos(dTheta_enc);
            dX_robot = (sinTheta / dTheta_enc) * forward - ((1 - cosTheta) / dTheta_enc) * lateral;
            dY_robot = ((1 - cosTheta) / dTheta_enc) * forward + (sinTheta / dTheta_enc) * lateral;
        }

        // Rotate to field frame using initial heading
        double cos = Math.cos(lastHeadingRad);
        double sin = Math.sin(lastHeadingRad);
        double dX_field = dX_robot * cos - dY_robot * sin;
        double dY_field = dX_robot * sin + dY_robot * cos;

        double fusedHeading = headingEnc;
        if (imu != null && USE_IMU && imuFailureCount < IMU_MAX_FAILURES) {
            try {
                // Use the correct SDK 8.0+ IMU API
                double imuYawRad = Math.toRadians(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
                imuYawRad *= IMU_YAW_SIGN;
                imuHeadingRad = Angle.norm(imuYawRad);
                fusedHeading = Angle.lerpAngle(headingEnc, imuHeadingRad, IMU_WEIGHT);
                imuFailureCount = 0; // Reset on successful read (allows recovery from glitches)
            } catch (Exception e) {
                // If IMU read fails, fall back to encoder-only heading
                // (Could be I2C timeout, sensor disconnected, etc.)
                // Count consecutive failures; only give up after IMU_MAX_FAILURES
                imuFailureCount++;
                fusedHeading = headingEnc;
            }
        }

        // integrate
        pose.x += dX_field;
        pose.y += dY_field;
        pose.heading = Angle.norm(fusedHeading);
        omega = dTheta_enc / dt;
        // robot-centric velocity from increments
        double vx_r = forward / dt;
        double vy_r = lateral / dt;
        // robot-centric velocities (not field frame)
        vx = vx_r;
        vy = vy_r;

        // --- writeActuators() none
        lastHeadingRad = pose.heading;
    }
}
