package com.teamcode.subsystems;

import static com.teamcode.Constants.*;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.teamcode.util.Angle;
import com.teamcode.util.Pose2d;

import java.util.List;
import com.qualcomm.hardware.lynx.LynxModule;

/**
 * Mecanum drive subsystem with integrated odometry-based pose control.
 * Provides both manual driving and autonomous pose-to-pose navigation.
 */
public class DriveSubsystem {
    private final DcMotorEx frontLeft, frontRight, backLeft, backRight;
    private final Odometry odometry;

    // PID gains for autonomous driving (from Constants)
    private double kPPos;
    private double kPAngle;
    private double maxPower;

    // Voltage compensation
    private final VoltageSensor voltageSensor;
    private static final double NOMINAL_VOLTAGE = 12.0;

    // Cached for zero-alloc pose reads
    private final Pose2d currentPose = new Pose2d();

    public DriveSubsystem(HardwareMap hw, Odometry odometry) {
        this.odometry = odometry;

        // Initialize gains from Constants
        this.kPPos = DRIVE_KP_POS;
        this.kPAngle = DRIVE_KP_ANGLE;
        this.maxPower = AUTO_MAX_SPEED;

        // Initialize voltage sensor for compensation
        voltageSensor = hw.voltageSensor.iterator().hasNext()
            ? hw.voltageSensor.iterator().next()
            : null;

        // Enable bulk caching for all hubs
        List<LynxModule> allHubs = hw.getAll(LynxModule.class);
        for (LynxModule module : allHubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        frontLeft  = hw.get(DcMotorEx.class, "front_left");
        frontRight = hw.get(DcMotorEx.class, "front_right");
        backLeft   = hw.get(DcMotorEx.class, "back_left");
        backRight  = hw.get(DcMotorEx.class, "back_right");

        // Standard mecanum motor directions
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);

        setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    /**
     * Manual robot-centric drive control.
     * @param forward  [-1, 1] forward velocity
     * @param strafe   [-1, 1] strafe velocity (left+)
     * @param turn     [-1, 1] turn velocity (CCW+)
     */
    public void driveRobotCentric(double forward, double strafe, double turn) {
        double fl = forward + strafe + turn;
        double fr = forward - strafe - turn;
        double bl = forward - strafe + turn;
        double br = forward + strafe - turn;

        // Normalize
        double max = Math.max(1.0, Math.max(Math.max(Math.abs(fl), Math.abs(fr)),
                                             Math.max(Math.abs(bl), Math.abs(br))));
        fl /= max;
        fr /= max;
        bl /= max;
        br /= max;

        setPowers(fl, fr, bl, br);
    }

    /**
     * Autonomous pose controller. Call every loop to drive toward target.
     * @param target Target pose (x, y, heading in radians)
     * @param posTol Position tolerance (inches)
     * @param headingTol Heading tolerance (radians)
     * @return true if within tolerances, false otherwise
     */
    public boolean driveToPose(Pose2d target, double posTol, double headingTol) {
        // Read current pose (zero-alloc)
        odometry.getPoseInto(currentPose);

        // Compute field-relative errors
        double errorX = target.x - currentPose.x;
        double errorY = target.y - currentPose.y;
        double errorHeading = Angle.shortestDiff(target.heading, currentPose.heading);

        // Check if at target
        double posError = Math.hypot(errorX, errorY);
        if (posError < posTol && Math.abs(errorHeading) < headingTol) {
            stop();
            return true;
        }

        // Transform field error to robot frame
        double cosH = Math.cos(currentPose.heading);
        double sinH = Math.sin(currentPose.heading);
        double errorForward = errorX * cosH + errorY * sinH;
        double errorStrafe  = -errorX * sinH + errorY * cosH;

        // Proportional control
        double forward = clamp(errorForward * kPPos, -maxPower, maxPower);
        double strafe  = clamp(errorStrafe * kPPos, -maxPower, maxPower);
        double turn    = clamp(errorHeading * kPAngle, -maxPower, maxPower);

        driveRobotCentric(forward, strafe, turn);
        return false;
    }

    /**
     * Drive forward slowly (for intake creep).
     * @param speed Forward speed [0, 1]
     */
    public void creepForward(double speed) {
        driveRobotCentric(speed, 0, 0);
    }

    /**
     * Stop all motors.
     */
    public void stop() {
        setPowers(0, 0, 0, 0);
    }

    /**
     * Set individual motor powers (clamped to [-1, 1]) with voltage compensation.
     */
    public void setPowers(double fl, double fr, double bl, double br) {
        // Apply voltage compensation to maintain consistent speed across battery discharge
        double voltageScale = 1.0;
        if (voltageSensor != null) {
            double voltage = voltageSensor.getVoltage();
            if (voltage > 0) {
                voltageScale = NOMINAL_VOLTAGE / voltage;
            }
        }

        frontLeft.setPower(clamp(fl * voltageScale, -1.0, 1.0));
        frontRight.setPower(clamp(fr * voltageScale, -1.0, 1.0));
        backLeft.setPower(clamp(bl * voltageScale, -1.0, 1.0));
        backRight.setPower(clamp(br * voltageScale, -1.0, 1.0));
    }

    public void setMode(DcMotor.RunMode mode) {
        frontLeft.setMode(mode);
        frontRight.setMode(mode);
        backLeft.setMode(mode);
        backRight.setMode(mode);
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior behavior) {
        frontLeft.setZeroPowerBehavior(behavior);
        frontRight.setZeroPowerBehavior(behavior);
        backLeft.setZeroPowerBehavior(behavior);
        backRight.setZeroPowerBehavior(behavior);
    }

    // Tuning methods
    public void setPositionGain(double kP) { this.kPPos = kP; }
    public void setHeadingGain(double kP) { this.kPAngle = kP; }
    public void setMaxPower(double max) { this.maxPower = max; }

    /**
     * Check if all drive motors are responding (detect disconnected motors).
     * @return true if all motors are accessible, false if any motor is disconnected
     */
    public boolean checkMotorHealth() {
        try {
            frontLeft.getCurrentPosition();
            frontRight.getCurrentPosition();
            backLeft.getCurrentPosition();
            backRight.getCurrentPosition();
            return true;
        } catch (Exception e) {
            return false;  // Motor disconnected or communication failure
        }
    }

    /**
     * Get current battery voltage (for telemetry/diagnostics).
     * @return voltage in volts, or 0.0 if sensor unavailable
     */
    public double getVoltage() {
        return (voltageSensor != null) ? voltageSensor.getVoltage() : 0.0;
    }

    /**
     * Robot-centric mecanum drive for TeleOp.
     * @param forward [-1, 1] forward/backward (negative = backward)
     * @param strafe [-1, 1] strafe left/right (positive = right)
     * @param turn [-1, 1] rotation (positive = CCW)
     * @param speedMultiplier [0, 1] global speed scaling (for precision mode)
     */
    public void teleopDrive(double forward, double strafe, double turn, double speedMultiplier) {
        // Apply deadzone
        if (Math.abs(forward) < JOYSTICK_DEADZONE) forward = 0.0;
        if (Math.abs(strafe) < JOYSTICK_DEADZONE) strafe = 0.0;
        if (Math.abs(turn) < JOYSTICK_DEADZONE) turn = 0.0;

        // Mecanum kinematics
        double fl = forward + strafe + turn;
        double fr = forward - strafe - turn;
        double bl = forward - strafe + turn;
        double br = forward + strafe - turn;

        // Normalize to [-1, 1]
        double max = Math.max(1.0, Math.max(Math.max(Math.abs(fl), Math.abs(fr)),
                                            Math.max(Math.abs(bl), Math.abs(br))));
        fl /= max;
        fr /= max;
        bl /= max;
        br /= max;

        // Apply speed multiplier and set powers
        setPowers(fl * speedMultiplier, fr * speedMultiplier,
                  bl * speedMultiplier, br * speedMultiplier);
    }

    private double clamp(double val, double min, double max) {
        return Math.max(min, Math.min(max, val));
    }
}
