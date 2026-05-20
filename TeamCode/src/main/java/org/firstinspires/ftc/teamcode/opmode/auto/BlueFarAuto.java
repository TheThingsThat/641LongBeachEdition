package org.firstinspires.ftc.teamcode.opmode.auto;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.configs.RobotConstants;
import org.firstinspires.ftc.teamcode.configs.Shooter;
import org.firstinspires.ftc.teamcode.configs.pedroPathing.Constants;
@Autonomous(name = "Blue Farzone Test")
public class BlueFarAuto {
    public static class Paths {
        private final Pose startPose = new Pose(10.887, 35.512, Math.toRadians(135)); // Start Pose of our robot.
        private final Pose scorePose = new Pose(60, 85, Math.toRadians(135)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
        private final Pose pickup1Pose = new Pose(37, 121, Math.toRadians(0)); // Highest (First Set) of Artifacts from the Spike Mark.
        private final Pose pickup2Pose = new Pose(43, 130, Math.toRadians(0)); // Middle (Second Set) of Artifacts from the Spike Mark.
        private final Pose pickup3Pose = new Pose(49, 135, Math.toRadians(0)); // Lowest (Third Set) of Artifacts from the Spike Mark.
    public PathChain MainChain;

    public Paths(Follower follower) {
        MainChain = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(50.000, 12.700),
                                new Pose(49.537, 35.108),
                                new Pose(10.887, 35.511)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180))
                .addPath(
                        new BezierLine(
                                new Pose(10.887, 35.511),
                                new Pose(49.043, 8.160)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .addPath(
                        new BezierLine(
                                new Pose(49.043, 8.160),
                                new Pose(5.689, 8.725)
                        )
                )
                .setTangentHeadingInterpolation()
                .addPath(
                        new BezierLine(
                                new Pose(5.689, 8.725),
                                new Pose(48.978, 8.055)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .addPath(
                        new BezierCurve(
                                new Pose(48.978, 8.055),
                                new Pose(18.321, 5.620),
                                new Pose(4.552, 13.005),
                                new Pose(6.548, 40.929)
                        )
                )
                .setTangentHeadingInterpolation()
                .addPath(
                        new BezierLine(
                                new Pose(6.548, 40.929),
                                new Pose(48.423, 8.252)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(135))
                .addPath(
                        new BezierCurve(
                                new Pose(48.423, 8.252),
                                new Pose(18.259, 5.512),
                                new Pose(5.373, 13.042),
                                new Pose(6.688, 40.896)
                        )
                )
                .setTangentHeadingInterpolation()
                .addPath(
                        new BezierLine(
                                new Pose(6.688, 40.896),
                                new Pose(48.220, 8.300)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(135))
                .addPath(
                        new BezierCurve(
                                new Pose(48.220, 8.300),
                                new Pose(18.274, 5.366),
                                new Pose(5.007, 12.663),
                                new Pose(6.980, 38.088)
                        )
                )
                .setTangentHeadingInterpolation()
                .addPath(
                        new BezierLine(
                                new Pose(6.980, 38.088),
                                new Pose(49.245, 8.188)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(135))
                .addPath(
                        new BezierLine(
                                new Pose(49.245, 8.188),
                                new Pose(11.149, 47.393)
                        )
                )
                .setTangentHeadingInterpolation()
                .build();
    }
}
}
