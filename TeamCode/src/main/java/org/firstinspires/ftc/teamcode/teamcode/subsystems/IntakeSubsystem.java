package com.teamcode.subsystems;

import static com.teamcode.Constants.*;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Simple intake subsystem for ball collection.
 * Controls a single motor or roller mechanism.
 * Gracefully handles missing motor (intake disconnected/misconfigured).
 * Supports outtake toggle mode for TeleOp.
 */
public class IntakeSubsystem {
    private final DcMotor intakeMotor;
    private boolean outtakeModeActive = false;
    private double lastPower = 0.0;

    public IntakeSubsystem(HardwareMap hw, String motorName) {
        DcMotor motor = null;
        try {
            motor = hw.get(DcMotor.class, motorName);
            motor.setDirection(DcMotorSimple.Direction.FORWARD);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        } catch (Exception e) {
            // Intake motor not found in config
            motor = null;
        }
        intakeMotor = motor;
    }

    /**
     * @return True if intake motor is present and functional
     */
    public boolean isPresent() {
        return intakeMotor != null;
    }

    /**
     * Turn intake on (collecting).
     */
    public void intakeOn() {
        if (intakeMotor != null) {
            intakeMotor.setPower(INTAKE_POWER_COLLECT);
        }
    }

    /**
     * Turn intake off.
     */
    public void intakeOff() {
        if (intakeMotor != null) {
            intakeMotor.setPower(0);
        }
    }

    /**
     * Reverse intake (eject).
     */
    public void reverse() {
        if (intakeMotor != null) {
            intakeMotor.setPower(INTAKE_POWER_EJECT);
        }
    }

    /**
     * Set custom intake power.
     */
    public void setPower(double power) {
        if (intakeMotor != null) {
            intakeMotor.setPower(power);
        }
    }

    /**
     * Toggle outtake mode (call on rising edge of dpad_up).
     */
    public void toggleOuttakeMode() {
        outtakeModeActive = !outtakeModeActive;
    }

    /**
     * Check if outtake mode is active.
     */
    public boolean isOuttakeModeActive() {
        return outtakeModeActive;
    }

    /**
     * Get direction sign for coupling with other motors.
     * @return +1.0 for intake (collect), -1.0 for outtake (eject), 0.0 for stopped
     */
    public double getDirectionSign() {
        if (intakeMotor == null || lastPower == 0.0) {
            return 0.0;
        }
        // Positive power = intake (collect), negative power = outtake (eject)
        return lastPower > 0 ? 1.0 : -1.0;
    }

    /**
     * Update intake state based on inputs (for TeleOp).
     * Uses power caching to prevent redundant setPower() calls.
     * @param intakeTrigger - LT value (0.0 to 1.0)
     */
    public void update(double intakeTrigger) {
        if (intakeMotor == null) return;

        // Determine target power based on mode
        double targetPower = 0.0;
        if (outtakeModeActive) {
            targetPower = INTAKE_POWER_EJECT;
        } else if (intakeTrigger > TRIGGER_THRESHOLD) {
            targetPower = INTAKE_POWER_COLLECT;
        }

        // Only update motor if power changed (avoid redundant I2C traffic)
        if (Math.abs(targetPower - lastPower) > 0.01) {
            intakeMotor.setPower(targetPower);
            lastPower = targetPower;
        }
    }

    /**
     * Get current power for telemetry.
     */
    public double getPower() {
        return (intakeMotor != null) ? intakeMotor.getPower() : 0.0;
    }
}
