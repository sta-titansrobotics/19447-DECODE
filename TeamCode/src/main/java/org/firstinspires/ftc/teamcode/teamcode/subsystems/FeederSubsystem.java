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

    public FeederSubsystem(HardwareMap hw) {
        feederMotor = hw.get(DcMotor.class, Constants.FEEDER_MOTOR_NAME);
        feederMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        feederMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        feederMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    /**
     * Command feeder to push ball (call while shoot button held).
     */
    public void setFeedCommand(boolean active) {
        // Detect rising edge to start ramp timer
        if (active && !wasFeeding) {
            rampTimer.reset();
        }
        feedCommandActive = active;
        wasFeeding = active;
    }

    /**
     * Update feeder motor power based on command state.
     * Call every loop.
     */
    public void update() {
        if (feedCommandActive) {
            // Ramp up smoothly over FEEDER_RAMP_TIME_MS to avoid jamming
            double elapsed = rampTimer.milliseconds();
            double rampFraction = Math.min(1.0, elapsed / Constants.FEEDER_RAMP_TIME_MS);
            double power = Constants.FEEDER_SHOOT_POWER * rampFraction;
            feederMotor.setPower(power);
        } else {
            feederMotor.setPower(0.0);
        }
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
