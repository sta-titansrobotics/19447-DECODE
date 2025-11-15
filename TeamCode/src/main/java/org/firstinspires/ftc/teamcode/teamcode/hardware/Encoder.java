package com.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Wrapper for odometry encoders plugged into motor ports.
 * Handles direction multipliers and delta tracking.
 */
public class Encoder {
    private static final int MAX_DELTA_TICKS = 10000; // Overflow protection

    private final DcMotorEx motor;
    private final int dir; // +1 or -1
    private int lastPos;
    private boolean initialized = false;

    public Encoder(HardwareMap hw, String name, int directionMultiplier) {
        this.motor = name == null || name.isEmpty() ? null : hw.get(DcMotorEx.class, name);
        this.dir = directionMultiplier;
        if (motor != null) {
            this.lastPos = motor.getCurrentPosition();
            this.initialized = true;
        }
    }

    public boolean isPresent() { return motor != null; }

    /** @return Current raw encoder position in ticks (direction-adjusted) */
    public int getRaw() { return isPresent() ? motor.getCurrentPosition() * dir : 0; }

    /**
     * @return Signed delta ticks since last call. Clamped to prevent overflow corruption.
     */
    public int getDeltaTicks() {
        if (!isPresent()) return 0;
        int cur = motor.getCurrentPosition() * dir;
        int dt = cur - lastPos;
        lastPos = cur;

        // Protect against massive deltas from encoder resets or I2C glitches
        if (Math.abs(dt) > MAX_DELTA_TICKS) {
            return 0; // Treat as invalid read
        }
        return dt;
    }

    public void reset() {
        if (!isPresent()) return;
        lastPos = motor.getCurrentPosition() * dir;
    }
}
