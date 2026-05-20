package org.firstinspires.ftc.teamcode.opmode.auto;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.bylazar.telemetry.PanelsTelemetry;

import org.firstinspires.ftc.teamcode.configs.pedroPathing.Constants;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.pedropathing.geometry.Pose;

@Autonomous(name = "Blue Far")
@Configurable // Panels
public class BlueFarAuto extends OpMode {
    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    private int pathState; // Current autonomous path state
    private Paths paths; // Paths defined in the Paths class

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(72, 8, Math.toRadians(90)));

        paths = new Paths(follower); // Build paths

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void start() {
        pathState = 0;
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
        panelsTelemetry.debug("Busy", follower.isBusy());
        panelsTelemetry.update(telemetry);
    }

    public static class Paths {
        public PathChain MainChain;

        public Paths(Follower follower) {
            MainChain = follower.pathBuilder()

                    .addPath(
                            new BezierCurve(
                                    new Pose(72.000, 8.000),
                                    new Pose(49.537, 35.108),
                                    new Pose(10.887, 35.511)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180))

                    .addPath(
                            new BezierLine(
                                    new Pose(10.887, 35.511),
                                    new Pose(49.778, 12.720)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(180))

                    .addPath(
                            new BezierLine(
                                    new Pose(49.778, 12.720),
                                    new Pose(7.796, 10.443)
                            )
                    )
                    .setTangentHeadingInterpolation()

                    .addPath(
                            new BezierLine(
                                    new Pose(7.796, 10.443),
                                    new Pose(50.155, 9.673)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(180))

                    .addPath(
                            new BezierCurve(
                                    new Pose(50.155, 9.673),
                                    new Pose(30.006, 8.678),
                                    new Pose(11.760, 16.535),
                                    new Pose(13.461, 39.605)
                            )
                    )
                    .setTangentHeadingInterpolation()

                    .addPath(
                            new BezierLine(
                                    new Pose(13.461, 39.605),
                                    new Pose(49.746, 13.106)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(135))

                    .addPath(
                            new BezierCurve(
                                    new Pose(49.746, 13.106),
                                    new Pose(30.173, 9.630),
                                    new Pose(11.620, 16.621),
                                    new Pose(13.454, 39.573)
                            )
                    )
                    .setTangentHeadingInterpolation()

                    .addPath(
                            new BezierLine(
                                    new Pose(13.454, 39.573),
                                    new Pose(49.838, 12.860)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(135))

                    .addPath(
                            new BezierCurve(
                                    new Pose(49.838, 12.860),
                                    new Pose(29.747, 9.191),
                                    new Pose(11.479, 16.634),
                                    new Pose(13.451, 39.706)
                            )
                    )
                    .setTangentHeadingInterpolation()

                    .addPath(
                            new BezierLine(
                                    new Pose(13.451, 39.706),
                                    new Pose(49.569, 12.356)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(135))

                    .addPath(
                            new BezierLine(
                                    new Pose(49.569, 12.356),
                                    new Pose(12.914, 46.658)
                            )
                    )
                    .setTangentHeadingInterpolation()

                    .build();
        }
    }

    public int autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(paths.MainChain, true);
                return 1;

            case 1:
                if (!follower.isBusy()) {
                    return 2;
                }
                return 1;

            case 2:
                return 2;
        }

        return pathState;
    }
}