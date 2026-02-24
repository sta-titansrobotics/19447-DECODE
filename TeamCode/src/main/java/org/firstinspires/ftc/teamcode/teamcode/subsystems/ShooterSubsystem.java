package com.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.teamcode.Constants;

public class ShooterSubsystem {
    public enum SpeedMode {
        LOW(Constants.SHOOTER_SPEED_LOW),
        MEDIUM(Constants.SHOOTER_SPEED_MEDIUM),
        MAX(Constants.SHOOTER_SPEED_MAX);

        public final double rpm;
        SpeedMode(double rpm) { this.rpm = rpm; }
    }

    private final DcMotorEx shooterMotor;
    private SpeedMode currentMode = SpeedMode.MEDIUM;
    private boolean shootCommandActive = false;
    private boolean shooterEnabled = true; // Can be disabled by A button

    // Cached target velocity (ticks/sec)
    private double targetVelocityTPS = 0.0;

    public ShooterSubsystem(HardwareMap hw) {
        shooterMotor = hw.get(DcMotorEx.class, Constants.SHOOTER_MOTOR_NAME);
        
        
        shooterMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT); // Coast for flywheel

        // Set PIDF for velocity control
        shooterMotor.setVelocityPIDFCoefficients(
            Constants.SHOOTER_KP,
            Constants.SHOOTER_KI,
            Constants.SHOOTER_KD,
            Constants.SHOOTER_KF
        );

        // Start at medium idle speed
        setTargetSpeed(currentMode);
    }

    /**
     * Set shooter speed mode (does not require shoot command to be active).
     * Shooter will idle at this speed.
     */
    public void setSpeedMode(SpeedMode mode) {
        if (mode != currentMode) {
            currentMode = mode;
            setTargetSpeed(mode);
        }
    }

    /**
     * Activate shooting (tightens control to exact target speed).
     * Call this while shoot button is held.
     */
    public void setShootCommand(boolean active) {
        shootCommandActive = active;
    }

    /**
     * Main update - call every loop to maintain velocity setpoint.
     * Uses negative velocity since motor direction is reversed.
     */
    public void update() {
        if (!shooterEnabled) {
            shooterMotor.setPower(0.0);
            return;
        }
        // Velocity control is handled by DcMotorEx internally via PIDF
        // Use negative velocity because motor direction is set to REVERSE
        shooterMotor.setVelocity(-targetVelocityTPS);
    }

    /**
     * Enable/disable shooter motor (called by A button).
     */
    public void setEnabled(boolean enabled) {
        shooterEnabled = enabled;
        if (!enabled) {
            shooterMotor.setPower(0.0);
        }
    }

    /**
     * Check if shooter is enabled.
     */
    public boolean isEnabled() {
        return shooterEnabled;
    }

    /**
     * Stop shooter (for emergency or end of match).
     */
    public void stop() {
        shooterMotor.setPower(0.0);
        shooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    /**
     * Check if shooter is at target speed (within tolerance).
     */
    public boolean isAtSpeed() {
        double currentRPM = getVelocityRPM();
        double targetRPM = currentMode.rpm;
        return Math.abs(currentRPM - targetRPM) < Constants.SHOOTER_VELOCITY_TOLERANCE_RPM;
    }

    /**
     * Get current shooter velocity in RPM.
     * Takes absolute value since motor direction is reversed.
     */
    public double getVelocityRPM() {
        double tps = Math.abs(shooterMotor.getVelocity()); // ticks/sec (absolute value for reversed motor)
        return tps / Constants.SHOOTER_RPM_TO_TPS;
    }

    /**
     * Get current speed mode.
     */
    public SpeedMode getSpeedMode() {
        return currentMode;
    }

    /**
     * Get target RPM for current mode.
     */
    public double getTargetRPM() {
        return currentMode.rpm;
    }

    /**
     * Check if shoot command is active.
     */
    public boolean isShootCommandActive() {
        return shootCommandActive;
    }

    // Internal helper to convert RPM to ticks/sec and set velocity
    private void setTargetSpeed(SpeedMode mode) {
        targetVelocityTPS = mode.rpm * Constants.SHOOTER_RPM_TO_TPS;
    }
}
