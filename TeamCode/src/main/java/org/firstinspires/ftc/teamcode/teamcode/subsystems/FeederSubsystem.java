package com.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.teamcode.Constants;

public class FeederSubsystem {
    private final DcMotor feederMotor;
    private final ElapsedTime rampTimer = new ElapsedTime();

    private boolean feedCommandActive = false;
    private boolean wasFeeding = false;
    private double rtValue = 0.0;
    private double intakeDirectionSign = 0.0;

    public FeederSubsystem(HardwareMap hw) {
        feederMotor = hw.get(DcMotor.class, Constants.FEEDER_MOTOR_NAME);
        feederMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        feederMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        feederMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    /**
     * Command feeder with RT trigger coupling to intake direction.
     * @param shootActive - shoot command (for backward compatibility)
     * @param rtTriggerValue - RT trigger value (0.0 to 1.0)
     * @param intakeDirSign - intake direction sign: +1.0 for intake, -1.0 for outtake, 0.0 for stopped
     */
    public void setFeedCommand(boolean shootActive, double rtTriggerValue, double intakeDirSign) {
        // Detect rising edge to start ramp timer
        boolean active = shootActive || (rtTriggerValue > Constants.TRIGGER_THRESHOLD);
        if (active && !wasFeeding) {
            rampTimer.reset();
        }
        feedCommandActive = shootActive; // Shoot command still uses ramp
        rtValue = rtTriggerValue;
        intakeDirectionSign = intakeDirSign;
        wasFeeding = active;
    }

    /**
     * Update feeder motor power based on command state.
     * RT motor couples to intake direction: same direction as intake, opposite for outtake.
     * Call every loop.
     */
    public void update() {
        double power = 0.0;
        
        if (feedCommandActive) {
            // Shoot command: ramp up smoothly over FEEDER_RAMP_TIME_MS to avoid jamming
            double elapsed = rampTimer.milliseconds();
            double rampFraction = Math.min(1.0, elapsed / Constants.FEEDER_RAMP_TIME_MS);
            power = Constants.FEEDER_SHOOT_POWER * rampFraction;
        } else if (rtValue > Constants.TRIGGER_THRESHOLD) {
            // RT-controlled: couple to intake direction if intake is active
            // If intake active (sign = +1), run forward (same direction as intake)
            // If outtake active (sign = -1), run backward (opposite direction of intake)
            // If intake not active (sign = 0), run forward by default
            if (intakeDirectionSign != 0.0) {
                power = rtValue * Constants.FEEDER_SHOOT_POWER * intakeDirectionSign;
            } else {
                // Intake not active, run forward by default
                power = rtValue * Constants.FEEDER_SHOOT_POWER;
            }
        }
        
        feederMotor.setPower(power);
    }

    /**
     * Stop feeder immediately.
     */
    public void stop() {
        feederMotor.setPower(0.0);
    }

    /**
     * Get current feeder power for telemetry.
     */
    public double getPower() {
        return feederMotor.getPower();
    }
}
