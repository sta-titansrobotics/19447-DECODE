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
     * Update intake state based on inputs (for TeleOp).
     * @param intakeTrigger - LT value (0.0 to 1.0)
     */
    public void update(double intakeTrigger) {
        if (intakeMotor == null) return;

        // Outtake mode takes priority
        if (outtakeModeActive) {
            reverse();
        } else if (intakeTrigger > TRIGGER_THRESHOLD) {
            intakeOn();
        } else {
            intakeOff();
        }
    }

    /**
     * Get current power for telemetry.
     */
    public double getPower() {
        return (intakeMotor != null) ? intakeMotor.getPower() : 0.0;
    }
}
