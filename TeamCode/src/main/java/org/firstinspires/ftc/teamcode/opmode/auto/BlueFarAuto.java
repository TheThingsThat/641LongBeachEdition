package org.firstinspires.ftc.teamcode.opmode.auto;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.bylazar.telemetry.PanelsTelemetry;

import org.firstinspires.ftc.teamcode.configs.RobotConstants;
import org.firstinspires.ftc.teamcode.configs.Shooter;
import org.firstinspires.ftc.teamcode.configs.pedroPathing.Constants;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.pedropathing.geometry.Pose;
import com.pedropathing.util.Timer;

@Autonomous(name = "BlueFarAuto", group = "Autonomous")
@Configurable // Panels
public class BlueFarAuto extends OpMode {
    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    private int pathState; // Current autonomous path state
    private Paths paths; // Paths defined in the Paths class

    // ── Shooter subsystem (turret auto-track + flywheel + hood) ────────────────
    private Shooter shooter;
    private final Pose goalPose = RobotConstants.blueGoal; // turret/flywheel/hood aim point

    // ── Intake ─────────────────────────────────────────────────────────────────
    // Runs forward at 1.0 the whole auto. After each kick finishes it reverses at
    // 1.0 for INTAKE_REVERSE_SEC (concurrent with the next path) to clear jams.
    private DcMotor intakeMotor;
    private Timer   intakeReverseTimer;
    private boolean intakeReversing = false;
    private static final double INTAKE_REVERSE_SEC = 0.2;

    // ── Timing / state-machine helpers ────────────────────────────────────────
    private Timer pathTimer;          // resets on every state change
    private boolean pathStarted;      // guards followPath()/startTripleKick() so they fire once per state

    // ── State constants (chassis movement only for now) ────────────────────────
    private static final int STATE_SHOOT_PRELOAD = 0; // sitting at start/shoot zone, fire preload
    private static final int STATE_PATH1 = 1;         // go to last row
    private static final int STATE_PATH2 = 2;         // return to shoot zone
    private static final int STATE_SHOOT1 = 3;        // fire last row balls
    private static final int STATE_PATH3 = 4;         // approach human player zone
    private static final int STATE_PATH4 = 5;         // human player zone pickup line (NEW)
    private static final int STATE_PATH5 = 6;         // return to shoot zone (human player)
    private static final int STATE_SHOOT2 = 7;        // fire human player balls
    private static final int STATE_PATH6 = 8;         // curving pickup
    private static final int STATE_PATH7 = 9;         // return to shoot zone
    private static final int STATE_SHOOT3 = 10;       // fire
    private static final int STATE_PATH8 = 11;        // curving pickup
    private static final int STATE_PATH9 = 12;        // return to shoot zone
    private static final int STATE_SHOOT4 = 13;       // fire
    private static final int STATE_PATH10 = 14;       // curving pickup
    private static final int STATE_PATH11 = 15;       // return to shoot zone
    private static final int STATE_SHOOT5 = 16;       // fire
    private static final int STATE_PATH12 = 17;       // drag / park
    private static final int STATE_DONE = 18;         // idle

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        pathTimer          = new Timer();
        intakeReverseTimer = new Timer();

        intakeMotor = hardwareMap.get(DcMotor.class, "intake");
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(51.690, 12.720, Math.toRadians(135))); // starts in the far-zone shoot area
        paths = new Paths(follower); // Build paths

        // Shooter encapsulates shooterR, shooterL, turretMotor, hoodServo and uses
        // the follower pose for live turret/flywheel/hood targeting.
        shooter = new Shooter(hardwareMap, follower, telemetry);
        shooter.resetTurretEncoder();
        // Seed the turret toward the goal from the starting pose so it doesn't slew
        // to robot-forward on the first loop tick.
        shooter.calculateTurretTarget(goalPose.getX(), goalPose.getY());

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void start() {
        setPathState(STATE_SHOOT_PRELOAD);
    }

    @Override
    public void loop() {
        follower.update();        // Update Pedro Pathing

        // ── Shooter: recompute all three targets from the live pose, then drive ──
        // calculateTargets() sets turretTargetTicks, targetFlywheelTPS, and
        // targetHoodServoPos in one call; the drive methods run their PIDs/setters.
        shooter.calculateTargets(goalPose);
        shooter.driveTurret();
        shooter.driveFlywheels(true); // flywheel spins the whole auto
        shooter.driveHood();
        shooter.updateTripleKick();   // advances any in-progress flywheel-gated triple kick

        handleIntake();           // forward 1.0, or reverse burst after a kick

        autonomousPathUpdate();   // Update autonomous state machine

        // Log values to Panels and Driver Station
        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        panelsTelemetry.debug("Busy", follower.isBusy());
        panelsTelemetry.update(telemetry);
    }

    /** Advance to a new state, resetting the timer + the followPath guard. */
    private void setPathState(int state) {
        pathState = state;
        pathTimer.resetTimer();
        pathStarted = false;
    }

    // ── Intake ─────────────────────────────────────────────────────────────────

    /** Drives the intake: forward 1.0 normally, reverse 1.0 during a post-kick burst. */
    private void handleIntake() {
        if (intakeReversing) {
            if (intakeReverseTimer.getElapsedTimeSeconds() >= INTAKE_REVERSE_SEC) {
                intakeReversing = false;
                intakeMotor.setPower(1.0);
            } else {
                intakeMotor.setPower(-1.0);
            }
        } else {
            intakeMotor.setPower(1.0);
        }
    }

    /** Starts a timed reverse-intake burst (runs concurrently with the next path). */
    private void startIntakeReverse() {
        intakeReversing = true;
        intakeReverseTimer.resetTimer();
    }

    public static class Paths {
        //individual chains
        public PathChain path1, path2, path3, path4, path5, path6,
                path7, path8, path9, path10, path11, path12;

        public Paths(Follower follower) {
            path1 = follower.pathBuilder() // path 1: last row
                    .addPath(new BezierCurve(
                            new Pose(51.690, 12.720),
                            new Pose(49.537, 35.108),
                            new Pose(16.476, 34.923)))
                    .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180))
                    .build();

            path2 = follower.pathBuilder() // path 2: return; shoot last row; ready for human player
                    .addPath(new BezierLine(
                            new Pose(16.476, 34.923),
                            new Pose(51.690, 12.720)))
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();

            path3 = follower.pathBuilder() // path 3: approach human player zone
                    .addPath(new BezierCurve(
                            new Pose(51.690, 12.720),
                            new Pose(33.500, 29.938),
                            new Pose(13.312, 21.181)))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(230))
                    .build();

            path4 = follower.pathBuilder() // path 4: human player zone pickup line (NEW)
                    .addPath(new BezierLine(
                            new Pose(13.312, 21.181),
                            new Pose(9.561, 11.620)))
                    .setLinearHeadingInterpolation(Math.toRadians(230), Math.toRadians(270))
                    .build();

            path5 = follower.pathBuilder() // path 5: return; shoot human player balls; ready for curving
                    .addPath(new BezierCurve(
                            new Pose(9.855, 10.149),
                            new Pose(30.328, 16.926),
                            new Pose(51.690, 12.720)))
                    .setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(135))
                    .build();

            path6 = follower.pathBuilder() // path 6: curving; pickup balls
                    .addPath(new BezierCurve(
                            new Pose(51.690, 12.720),
                            new Pose(24.789, 8.159),
                            new Pose(13.532, 16.621),
                            new Pose(13.454, 41.573)))
                    .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(90))
                    .build();

            path7 = follower.pathBuilder() // path 7: return shooting zone; shoot
                    .addPath(new BezierLine(
                            new Pose(13.454, 41.573),
                            new Pose(51.690, 12.700)))
                    .setConstantHeadingInterpolation(Math.toRadians(135))
                    .build();

            path8 = follower.pathBuilder() // path 8: curving; pickup balls
                    .addPath(new BezierCurve(
                            new Pose(51.690, 12.700),
                            new Pose(24.789, 8.159),
                            new Pose(13.532, 16.621),
                            new Pose(13.454, 41.573)))
                    .setTangentHeadingInterpolation()
                    .build();

            path9 = follower.pathBuilder() // path 9: return shooting zone; shoot
                    .addPath(new BezierLine(
                            new Pose(13.454, 41.573),
                            new Pose(51.690, 12.720)))
                    .setConstantHeadingInterpolation(Math.toRadians(135))
                    .build();

            path10 = follower.pathBuilder() // path 10: curving; pickup balls
                    .addPath(new BezierCurve(
                            new Pose(51.690, 12.720),
                            new Pose(24.789, 8.159),
                            new Pose(13.532, 16.621),
                            new Pose(13.454, 41.573)))
                    .setTangentHeadingInterpolation()
                    .build();

            path11 = follower.pathBuilder() // path 11: return shooting zone; shoot
                    .addPath(new BezierLine(
                            new Pose(13.454, 41.573),
                            new Pose(51.690, 12.720)))
                    .setConstantHeadingInterpolation(Math.toRadians(135))
                    .build();

            path12 = follower.pathBuilder() // path 12: drag end
                    .addPath(new BezierLine(
                            new Pose(51.690, 12.720),
                            new Pose(38.949, 21.211)))
                    .setTangentHeadingInterpolation()
                    .build();
        }
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            // ── Preload shot (already sitting in the shoot zone) ───────────────
            case STATE_SHOOT_PRELOAD:
                if (!pathStarted) {
                    shooter.startTripleKick(true);
                    pathStarted = true;
                } else if (!shooter.isTripleKicking()) {
                    startIntakeReverse();
                    setPathState(STATE_PATH1);
                }
                break;

            // ── Cycle 1: last row ──────────────────────────────────────────────
            case STATE_PATH1:
                if (!pathStarted) { follower.followPath(paths.path1, true); pathStarted = true; }
                if (!follower.isBusy()) setPathState(STATE_PATH2);
                break;

            case STATE_PATH2:
                if (!pathStarted) { follower.followPath(paths.path2, true); pathStarted = true; }
                if (!follower.isBusy()) setPathState(STATE_SHOOT1);
                break;

            case STATE_SHOOT1:
                if (!pathStarted) {
                    shooter.startTripleKick(true);
                    pathStarted = true;
                } else if (!shooter.isTripleKicking()) {
                    startIntakeReverse();
                    setPathState(STATE_PATH3);
                }
                break;

            // ── Cycle 2: human player zone ─────────────────────────────────────
            case STATE_PATH3: // approach human player zone
                if (!pathStarted) { follower.followPath(paths.path3, true); pathStarted = true; }
                if (!follower.isBusy()) setPathState(STATE_PATH4);
                break;

            case STATE_PATH4: // human player zone pickup line (NEW)
                if (!pathStarted) { follower.followPath(paths.path4, true); pathStarted = true; }
                if (!follower.isBusy()) setPathState(STATE_PATH5);
                break;

            case STATE_PATH5: // return; ready to shoot human player balls
                if (!pathStarted) { follower.followPath(paths.path5, true); pathStarted = true; }
                if (!follower.isBusy()) setPathState(STATE_SHOOT2);
                break;

            case STATE_SHOOT2:
                if (!pathStarted) {
                    shooter.startTripleKick(true);
                    pathStarted = true;
                } else if (!shooter.isTripleKicking()) {
                    startIntakeReverse();
                    setPathState(STATE_PATH6);
                }
                break;

            // ── Cycle 3: curving pickup ────────────────────────────────────────
            case STATE_PATH6:
                if (!pathStarted) { follower.followPath(paths.path6, 0.75, true); pathStarted = true; }
                if (!follower.isBusy()) setPathState(STATE_PATH7);
                break;

            case STATE_PATH7:
                if (!pathStarted) { follower.followPath(paths.path7, true); pathStarted = true; }
                if (!follower.isBusy()) setPathState(STATE_SHOOT3);
                break;

            case STATE_SHOOT3:
                if (!pathStarted) {
                    shooter.startTripleKick(true);
                    pathStarted = true;
                } else if (!shooter.isTripleKicking()) {
                    startIntakeReverse();
                    setPathState(STATE_PATH8);
                }
                break;

            // ── Cycle 4: curving pickup ────────────────────────────────────────
            case STATE_PATH8:
                if (!pathStarted) { follower.followPath(paths.path8, 0.75, true); pathStarted = true; }
                if (!follower.isBusy()) setPathState(STATE_PATH9);
                break;

            case STATE_PATH9:
                if (!pathStarted) { follower.followPath(paths.path9, true); pathStarted = true; }
                if (!follower.isBusy()) setPathState(STATE_SHOOT4);
                break;

            case STATE_SHOOT4:
                if (!pathStarted) {
                    shooter.startTripleKick(true);
                    pathStarted = true;
                } else if (!shooter.isTripleKicking()) {
                    startIntakeReverse();
                    setPathState(STATE_PATH10);
                }
                break;

            // ── Cycle 5: curving pickup ────────────────────────────────────────
            case STATE_PATH10:
                if (!pathStarted) { follower.followPath(paths.path10, 0.75, true); pathStarted = true; }
                if (!follower.isBusy()) setPathState(STATE_PATH11);
                break;

            case STATE_PATH11:
                if (!pathStarted) { follower.followPath(paths.path11, true); pathStarted = true; }
                if (!follower.isBusy()) setPathState(STATE_SHOOT5);
                break;

            case STATE_SHOOT5:
                if (!pathStarted) {
                    shooter.startTripleKick(true);
                    pathStarted = true;
                } else if (!shooter.isTripleKicking()) {
                    startIntakeReverse();
                    setPathState(STATE_PATH12);
                }
                break;

            // ── Park / drag end ────────────────────────────────────────────────
            case STATE_PATH12:
                if (!pathStarted) { follower.followPath(paths.path12, true); pathStarted = true; }
                if (!follower.isBusy()) setPathState(STATE_DONE);
                break;

            case STATE_DONE:
            default:
                break;
        }
    }
}
