package com.teamcode;

/*
 * ╔══════════════════════════════════════════════════════════════════════════╗
 * ║          ROBOT CONFIGURATION CHECKLIST - FTC DRIVER STATION              ║
 * ╚══════════════════════════════════════════════════════════════════════════╝
 *
 * Copy these device names EXACTLY into Driver Station Robot Configuration.
 * Spelling and capitalization must match perfectly!
 *
 * CONTROL HUB / EXPANSION HUB:
 * ┌────────────────────────────────────────────────────────────────────────┐
 * │ Motors (DcMotorEx):                                                    │
 * │   ✓ front_left    - Mecanum drive (REV/goBILDA motor)                 │
 * │   ✓ front_right   - Mecanum drive (REV/goBILDA motor)                 │
 * │   ✓ back_left     - Mecanum drive (REV/goBILDA motor)                 │
 * │   ✓ back_right    - Mecanum drive (REV/goBILDA motor)                 │
 * │   ✓ odo_left      - Odometry encoder (goBILDA 8192 CPR Through-Bore)  │
 * │   ✓ odo_right     - Odometry encoder (goBILDA 8192 CPR Through-Bore)  │
 * │   ○ odo_strafe    - Strafe encoder [OPTIONAL - for 3-wheel odometry]  │
 * │   ○ intake        - Intake motor (DcMotor, any FTC-legal motor)        │
 * │   ○ shooter       - Shooter flywheel (DcMotorEx with encoder)          │
 * │   ○ m4            - Feeder motor (DcMotor, pushes balls to shooter)    │
 * │                                                                        │
 * │ IMU:                                                                   │
 * │   ✓ imu           - REV Internal IMU (BHI260AP)                        │
 * │                                                                        │
 * │ Camera (USB):                                                          │
 * │   ○ Webcam 1      - UVC webcam (Logitech C270/C920 recommended)       │
 * └────────────────────────────────────────────────────────────────────────┘
 *
 * WIRING NOTES:
 *   • Odometry encoders plug into motor ports (we only read position)
 *   • Use short, shielded cables for encoders to reduce I2C noise
 *   • IMU orientation: Logo UP, USB port facing FORWARD
 *   • Test EACH device in Driver Station before running OpModes
 *
 * CALIBRATION PROCEDURE:
 *   1. TRACK_WIDTH_IN: Measure center-to-center distance between left/right odo wheels
 *   2. LATERAL_WHEEL_OFFSET_IN: Measure from robot center to strafe wheel
 *   3. WHEEL_DIAMETER_IN: Measure odo wheel diameter with calipers
 *   4. TICKS_PER_REV: Verify encoder spec (goBILDA 8192 CPR = 8192 ticks/rev)
 *   5. Test odometry by pushing robot 12" forward → check telemetry shows ~12"
 */

public final class Constants {
    private Constants() {}

    // ========== HARDWARE DEVICE NAMES (must match Robot Configuration) ==========
    public static final String ENC_LEFT   = "odo_left";
    public static final String ENC_RIGHT  = "odo_right";
    public static final String ENC_STRAFE = "odo_strafe"; // set null/"" to disable (2-wheel mode)
    public static final String IMU_NAME   = "imu";

    // Encoder configuration
    // For goBILDA 8192 CPR through-bore: 8192 ticks per wheel revolution.
    public static final double TICKS_PER_REV = 8192.0;
    public static final double WHEEL_DIAMETER_IN = 2.0;     // inches
    public static final double GEAR_RATIO = 1.0;            // wheel revs per encoder rev

    // Geometry (inches)
    public static final double TRACK_WIDTH_IN = 13.5;       // distance between left/right odometry wheels
    public static final double LATERAL_WHEEL_OFFSET_IN = 7.5; // strafe wheel offset from robot center

    // Encoder direction multipliers (+1 or -1) to make forward = +X, left wheel increasing forward
    public static final int LEFT_DIR   = +1;
    public static final int RIGHT_DIR  = -1;
    public static final int STRAFE_DIR = +1;

    // IMU fusion
    public static final boolean USE_IMU = true;     // set false to use pure encoder heading
    public static final double IMU_WEIGHT = 0.12;   // 0..1 complementary filter weight toward IMU absolute yaw
    public static final double IMU_YAW_SIGN = +1.0; // flip if yaw sign is inverted

    // Odometry update
    public static final double MAX_DT_S = 0.050; // cap dt if loops stall (50ms)

    // Telemetry
    public static final boolean TELEMETRY_VERBOSE = true;

    // ========== AUTONOMOUS CONFIGURATION ==========
    // Starting pose (right side of field, facing forward)
    public static final double AUTO_START_X = 12.0;             // inches from field origin
    public static final double AUTO_START_Y = -62.0;            // right side (negative y)
    public static final double AUTO_START_HEADING_RAD = Math.toRadians(90); // facing up-field

    // Scoring position (e.g., near backdrop/basket)
    public static final double AUTO_SCORE_X = 36.0;             // forward toward scoring area
    public static final double AUTO_SCORE_Y = -36.0;            // adjusted right side
    public static final double AUTO_SCORE_HEADING_RAD = Math.toRadians(135); // angle toward scoring

    // Parking position (safe zone)
    public static final double AUTO_PARK_X = 48.0;              // further forward
    public static final double AUTO_PARK_Y = -60.0;             // back to edge
    public static final double AUTO_PARK_HEADING_RAD = Math.toRadians(90); // facing forward

    // Autonomous control gains
    public static final double DRIVE_KP_POS = 0.04;             // proportional gain for position error (in/s per in)
    public static final double DRIVE_KP_ANGLE = 2.0;            // proportional gain for heading error (rad/s per rad)
    public static final double AUTO_MAX_SPEED = 0.6;            // max drive power during auto [0, 1]
    public static final double AUTO_POS_TOLERANCE_IN = 2.0;     // position error tolerance (inches)
    public static final double AUTO_ANGLE_TOLERANCE_RAD = Math.toRadians(5); // heading error tolerance

    // Autonomous timing
    public static final double AUTO_STEP_TIMEOUT_S = 5.0;       // max time per state before forcing next
    public static final double AUTO_SCORE_DURATION_S = 1.5;     // time to hold scoring position

    // ========== APRILTAG AUTONOMOUS (BALL ROUTE) ==========
    // Device names
    public static final String INTAKE_MOTOR_NAME = "intake";
    public static final String WEBCAM_NAME = "Webcam 1";

    // Starting pose for AprilTag auto
    public static final double APRILTAG_AUTO_START_X = 0.0;
    public static final double APRILTAG_AUTO_START_Y = 0.0;
    public static final double APRILTAG_AUTO_START_HEADING = 0.0; // radians

    // Ball pickup locations (adjust per field setup)
    public static final double BALL_PICKUP_X = 36.0;
    public static final double BALL_PICKUP_Y = 0.0;
    public static final double BALL_PICKUP_HEADING = 0.0;

    // Scoring locations for each path
    // LEFT path (Tag 1)
    public static final double SCORE_LEFT_X = 60.0;
    public static final double SCORE_LEFT_Y = 24.0;
    public static final double SCORE_LEFT_HEADING = Math.toRadians(90);

    // CENTER path (Tag 2 or default)
    public static final double SCORE_CENTER_X = 60.0;
    public static final double SCORE_CENTER_Y = 0.0;
    public static final double SCORE_CENTER_HEADING = Math.toRadians(90);

    // RIGHT path (Tag 3)
    public static final double SCORE_RIGHT_X = 60.0;
    public static final double SCORE_RIGHT_Y = -24.0;
    public static final double SCORE_RIGHT_HEADING = Math.toRadians(90);

    // Parking locations (per path)
    public static final double PARK_LEFT_X = 48.0;
    public static final double PARK_LEFT_Y = 36.0;
    public static final double PARK_LEFT_HEADING = Math.toRadians(0);

    public static final double PARK_CENTER_X = 48.0;
    public static final double PARK_CENTER_Y = 0.0;
    public static final double PARK_CENTER_HEADING = Math.toRadians(0);

    public static final double PARK_RIGHT_X = 48.0;
    public static final double PARK_RIGHT_Y = -36.0;
    public static final double PARK_RIGHT_HEADING = Math.toRadians(0);

    // Drive tolerances for AprilTag auto
    public static final double APRILTAG_POS_TOL_IN = 3.0;
    public static final double APRILTAG_HEADING_TOL_RAD = Math.toRadians(10);

    // Intake timing
    public static final double INTAKE_CREEP_DURATION_S = 1.5;
    public static final double INTAKE_CREEP_SPEED = 0.3;

    // State timeouts
    public static final double DRIVE_TIMEOUT_S = 6.0;
    public static final double INTAKE_TIMEOUT_S = 3.0;
    public static final double SCORE_TIMEOUT_S = 2.0;
    public static final double PARK_TIMEOUT_S = 5.0;

    // ========== VISION / APRILTAG DETECTION ==========
    // AprilTag ID mappings
    public static final int APRILTAG_ID_LEFT = 1;
    public static final int APRILTAG_ID_CENTER = 2;
    public static final int APRILTAG_ID_RIGHT = 3;

    // Detection thresholds
    public static final double APRILTAG_MIN_CONFIDENCE = 0.7;
    public static final double APRILTAG_DETECTION_TIMEOUT_S = 2.0;
    public static final double APRILTAG_TAG_SWITCH_MARGIN = 0.2; // Hysteresis for multi-tag stability

    // ========== INTAKE POWER LEVELS ==========
    public static final double INTAKE_POWER_COLLECT = 0.8;
    public static final double INTAKE_POWER_EJECT = -0.6;

    // ========== TELEOP HARDWARE ==========
    public static final String SHOOTER_MOTOR_NAME = "shooter";
    public static final String FEEDER_MOTOR_NAME = "m4";

    // ========== SHOOTER CONFIGURATION ==========
    // Flywheel speeds in RPM
    // For REV HD Hex Motor: ~2800 RPM max
    // Adjust based on actual motor specs
    public static final double SHOOTER_SPEED_LOW = 1200.0;      // RPM (conservative for close shots)
    public static final double SHOOTER_SPEED_MEDIUM = 1800.0;   // RPM (default idle)
    public static final double SHOOTER_SPEED_MAX = 2400.0;      // RPM (max power shots)

    // Convert RPM to encoder ticks/sec (goBILDA 5202 motor: 537.7 PPR)
    public static final double SHOOTER_TICKS_PER_REV = 537.7;
    public static final double SHOOTER_RPM_TO_TPS = SHOOTER_TICKS_PER_REV / 60.0;

    // PIDF gains for shooter velocity control (tune on actual hardware)
    public static final double SHOOTER_KP = 5.0;
    public static final double SHOOTER_KI = 0.1;
    public static final double SHOOTER_KD = 0.0;
    public static final double SHOOTER_KF = 12.0;  // Feedforward

    // Shooter tolerance for "at speed" detection
    public static final double SHOOTER_VELOCITY_TOLERANCE_RPM = 50.0; // within 50 RPM = ready

    // ========== FEEDER (M4) CONFIGURATION ==========
    public static final double FEEDER_SHOOT_POWER = 0.5;  // Half speed during shooting
    public static final double FEEDER_RAMP_TIME_MS = 150; // Ramp up time to avoid jamming

    // ========== TELEOP CONTROL THRESHOLDS ==========
    public static final double TRIGGER_THRESHOLD = 0.1;  // Minimum trigger press to activate
    public static final double JOYSTICK_DEADZONE = 0.05; // Ignore tiny joystick drift

    // ========== DRIVE SPEEDS ==========
    public static final double TELEOP_DRIVE_SPEED_NORMAL = 1.0;  // Full speed
    public static final double TELEOP_DRIVE_SPEED_PRECISION = 0.4; // Slow mode (if bumper held)
}
