package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.Encoder;
import com.pedropathing.ftc.localization.constants.ThreeWheelConstants;
import com.pedropathing.ftc.localization.constants.TwoWheelConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(11.43053)
            .forwardZeroPowerAcceleration(-60.53188108986048)
            .lateralZeroPowerAcceleration(-70.992092629844)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.08, 0, 0.001, 0.025))
            .headingPIDFCoefficients(new PIDFCoefficients(0.9, 0, 0.002, 0.004))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.6, 0.0, 0.0001, 0.6, 0.025))
            .centripetalScaling(0.0006);

    public static PathConstraints pathConstraints = new PathConstraints(0.99,
            1.0,
            1,
            1);

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("frontRight")
            .rightRearMotorName("backRight")
            .leftRearMotorName("backLeft")
            .leftFrontMotorName("frontLeft")
            .leftFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .leftRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .xVelocity(63.29456431471457)
            .yVelocity(50.167069562776824);

    public static TwoWheelConstants localizerConstants = new TwoWheelConstants()
            .forwardPodY(0.0)
            .strafePodX(-4.875)
            .forwardTicksToInches(0.001955168116108304)
            .strafeTicksToInches(0.0019967819235301)
            .forwardEncoder_HardwareMapName("frontRight")
            .strafeEncoder_HardwareMapName("backRight")
            .forwardEncoderDirection(Encoder.REVERSE)
            .strafeEncoderDirection(Encoder.REVERSE)
            .IMU_HardwareMapName("imu")
            .IMU_Orientation(
                    new RevHubOrientationOnRobot(
                            RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                            RevHubOrientationOnRobot.UsbFacingDirection.UP
                    )
            );

/*
    public static ThreeWheelConstants localizerConstants = new ThreeWheelConstants()
            .forwardTicksToInches(.000587155572144018)
            .strafeTicksToInches(.0005883031882140352)
            .turnTicksToInches(.0005826444987816472)
            .leftPodY(5.25)
            .rightPodY(-5.75)
            .strafePodX(-1.0)
            .leftEncoder_HardwareMapName("frontRightMotor")
            .rightEncoder_HardwareMapName("backRightMotor")
            .strafeEncoder_HardwareMapName("frontLeftMotor")
            .leftEncoderDirection(Encoder.FORWARD)
            .rightEncoderDirection(Encoder.FORWARD)
            .strafeEncoderDirection(Encoder.REVERSE);

*/
    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .twoWheelLocalizer(localizerConstants)
                //.threeWheelLocalizer(localizerConstants)
                .build();
    }
}
