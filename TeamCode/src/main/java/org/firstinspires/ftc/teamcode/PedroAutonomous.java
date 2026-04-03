package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.bylazar.telemetry.PanelsTelemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Pedro Pathing Autonomous", group = "Autonomous")
@Configurable // Panels
public class PedroAutonomous extends OpMode {
    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    private int pathState; // Current autonomous path state (state machine)
    private Paths paths; // Paths defined in the Paths class

    private ElapsedTime pathTimer;

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(122.00, 122.00, Math.toRadians(0)));

        paths = new Paths(follower); // Build paths
        pathTimer = new ElapsedTime();

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void start() {
        // Kick off the state machine when the driver hits PLAY
        setPathState(2);
    }

    @Override
    public void loop() {
        follower.update(); // Update Pedro Pathing
        pathState = autonomousPathUpdate(); // Update autonomous state machine

        // Log values to Panels and Driver Station
        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        panelsTelemetry.update(telemetry);
    }

    public static class Paths {
        public PathChain Path1;
        public PathChain Path2;
        public PathChain Path3;
        public PathChain Path4;
        public PathChain Path5;
        public PathChain Path6;
        public PathChain Path7;
        public PathChain Path8;
        public PathChain Path9;
        public PathChain Path10;
        public PathChain Path11;
        public PathChain Path12;

        public Paths(Follower follower) {
            Path1 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(122.000, 122.000),
                                    new Pose(122.000, 122.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(40))
                    .build();

            Path2 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(122.000, 122.000),
                                    new Pose(96.841, 95.848)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(40))
                    .build();

            Path3 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(96.841, 95.848),
                                    new Pose(96.890, 83.441)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(0))
                    .build();

            Path4 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(96.890, 83.441),
                                    new Pose(128.662, 83.545)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(0))
                    .build();

            Path5 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(128.662, 83.545),
                                    new Pose(96.662, 96.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(40))
                    .build();

            Path6 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(96.662, 96.000),
                                    new Pose(100.979, 59.579)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(40), Math.toRadians(0))
                    .build();

            Path7 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(100.979, 59.579),
                                    new Pose(128.648, 59.634)
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            Path8 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(128.648, 59.634),
                                    new Pose(96.759, 95.759)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(40))
                    .build();

            Path9 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(96.759, 95.759),
                                    new Pose(101.793, 34.897)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(40), Math.toRadians(0))
                    .build();

            Path10 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(101.793, 34.897),
                                    new Pose(128.848, 35.110)
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            Path11 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(128.848, 35.110),
                                    new Pose(96.621, 95.462)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(40))
                    .build();

            Path12 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(96.621, 95.462),
                                    new Pose(126.462, 69.993)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(40), Math.toRadians(90))
                    .build();
        }
    }

    public int autonomousPathUpdate() {
        switch (pathState) {
            case 1:
                // Start following Path 1
                follower.followPath(paths.Path1);
                setPathState(2);
                break;
            case 2:
                // Wait for the follower to finish Path 1 before starting Path 2
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path2);
                    setPathState(3);
                }
                break;
            case 3:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path3);
                    setPathState(4);
                }
                break;
            case 4:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path4);
                    setPathState(5);
                }
                break;
            case 5:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path5);
                    setPathState(6);
                }
                break;
            case 6:
                if (pathTimer.seconds() > 3.5) {
                    follower.followPath(paths.Path6);
                    setPathState(7);
                }
                /*
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path6);
                    setPathState(7);
                }
                */
                break;
            case 7:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path7);
                    setPathState(8);
                }
                break;
            case 8:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path8);
                    setPathState(9);
                }
                break;
            case 9:
                if (pathTimer.seconds() > 4.5) {
                    follower.followPath(paths.Path9);
                    setPathState(10);
                }
                /*
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path9);
                    setPathState(10);
                }
                */
                break;
            case 10:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path10);
                    setPathState(11);
                }
                break;
            case 11:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path11);
                    setPathState(12);
                }
                break;
            case 12:
                if (pathTimer.seconds() > 5.5) {
                    follower.followPath(paths.Path12);
                    setPathState(13);
                }
                /*
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path12);
                    setPathState(13); // Transition to a finished state
                }
                */
                break;
            case 13:
                // The robot has finished all paths. We do nothing here.
                if (!follower.isBusy()) {
                    // You could put a final stop command here if needed
                }
                break;
        }
        return pathState;
    }

    // Helper method to transition states and reset the timer
    public void setPathState(int state) {
        pathState = state;
        pathTimer.reset();
    }
}