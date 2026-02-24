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

    // Test seam: optional injected reader to allow unit tests to run without SDK
    public interface MotorReader {
        int getCurrentPosition();
    }

    private final MotorReader testReader;

    public Encoder(HardwareMap hw, String name, int directionMultiplier) {
        this(hw, name, directionMultiplier, null);
    }

    // Package-private constructor for tests to inject MotorReader
    Encoder(HardwareMap hw, String name, int directionMultiplier, MotorReader reader) {
        DcMotorEx m = null;
        try {
            m = (name == null || name.isEmpty()) ? null : hw.get(DcMotorEx.class, name);
        } catch (Exception e) {
            // Motor not found in config - encoder will be disabled
            m = null;
        }
        this.motor = m;
        this.dir = directionMultiplier;
        this.testReader = reader;
        if (motor != null) {
            try {
                this.lastPos = (testReader != null ? testReader.getCurrentPosition() : motor.getCurrentPosition()) * dir;
                this.initialized = true;
            } catch (Exception e) {
                // Failed to read encoder - treat as not present
                this.initialized = false;
            }
        } else if (testReader != null) {
            // Test seam: allow tests to create an Encoder without a real motor
            this.lastPos = testReader.getCurrentPosition() * dir;
            this.initialized = true;
        }
    }

    public boolean isPresent() { return initialized && (motor != null || testReader != null); }

    /** @return Current raw encoder position in ticks (direction-adjusted) */
    public int getRaw() { return isPresent() ? motor.getCurrentPosition() * dir : 0; }

    /**
     * @return Signed delta ticks since last call. Clamped to prevent overflow corruption.
     */
    public int getDeltaTicks() {
        if (!isPresent()) return 0;
        int cur = (testReader != null ? testReader.getCurrentPosition() : motor.getCurrentPosition()) * dir;
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
