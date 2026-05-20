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

@Autonomous(name = "Blue 18 ball normal")
public class BlueNearAuto extends OpMode {
    // ── Shooter velocity PID state ────────────────────────────────────────────
    private double pidIntegral   = 0.0;
    private double pidLastError  = 0.0;
    private int    lastEncoderPos = 0;
    private long   lastTimeNs    = 0;
    // ── States ───────────────────────────────────────────────────────────────────
    // State constants for clarity
    private static final int STATE_INIT = 0;
    private static final int STATE_MOVE_TO_SCORE_FIRST = 1;
    private static final int STATE_SHOOT_FIRST = 2;

    // Pickup 3 cycle
    private static final int STATE_MOVE_TO_PICKUP2 = 3;
    private static final int STATE_MOVE_TO_PICKUP3_PART2 = 4;
    private static final int STATE_MOVE_TO_SCORE2 = 5;
    private static final int STATE_SHOOT_SECOND = 6;

    // Pickup 2 cycle
    private static final int STATE_MOVE_TO_GATE = 7;
    private static final int STATE_MOVE_TO_SCORE_GATE = 9;
    private static final int STATE_SHOOT_GATE = 10;

    private static final int STATE_MOVE_TO_GATE2= 8;
    private static final int STATE_MOVE_TO_SCORE_GATE2 = 17;
    private static final int STATE_SHOOT_GATE2 = 18;
    private static final int STATE_MOVE_TO_GATE3 = 19;
    private static final int STATE_MOVE_TO_SCORE_GATE3 = 20;
    private static final int STATE_SHOOT_GATE3 = 21;


    // Pickup 1 cycle
    private static final int STATE_MOVE_TO_PICKUP1 = 11;
    private static final int STATE_MOVE_TO_PICKUP1_PART2 = 12;
    private static final int STATE_MOVE_TO_SCORE1 = 13;
    private static final int STATE_SHOOT_FIRST_PICKUP = 14;

    private static final int STATE_PARK = 15;

    private static final int STATE_DONE = 16;
    // ── Hardware ─────────────────────────────────────────────────────────────────
    private Shooter shooter;
    private DcMotor toggleMotor,shooterR,shooterL;
    private Servo   servo;

    private Follower follower;

    // ── Paths ────────────────────────────────────────────────────────────────────
    private PathChain gateRotate;
    private PathChain pickupGate, scoreGate, pickupSecond;
    private PathChain score2, score3, scoreFirst, pickupFirst;

    // ── State machine ────────────────────────────────────────────────────────────
    private int     pathState;
    private boolean pathStarted = false;

    // ── Servo positions ───────────────────────────────────────────────────────────
    private static final double SERVO_INTAKE = 0.35;
    private static final double SERVO_SHOOT  = 0.51;

    // ── Shoot-while-moving ────────────────────────────────────────────────────────
    private static final double SHOOT_TRIGGER_DIST = 5.0;
    private boolean shootingStarted = false;
    private Timer   shootTimer;

    // ── Gate pickup helpers ───────────────────────────────────────────────────────
    private boolean atGate     = false;
    private Timer   shakeTimer;

    // ── Timers ────────────────────────────────────────────────────────────────────
    private Timer pathTimer, opmodeTimer;

    // ── Poses (mirrored to red side) ──────────────────────────────────────────────
    private final Pose startPose    = new Pose(24.86, 121.8,  Math.toRadians(180));
    private final Pose control1     = new Pose(43.29, 104.73, Math.toRadians(180));
    private final Pose scorePose    = new Pose(52,    79.2,   Math.toRadians(180));

    private final Pose control3     = new Pose(59.221, 79.143);
    private final Pose pickup3Pose2 = new Pose(25.2,   83,  Math.toRadians(180));

    //private final Pose control2     = new Pose(56.73,  54.9).mirror();
    private final Pose control2     = new Pose(58,  57.6);

    private final Pose pickup2Pose2 = new Pose(24.7,   58, Math.toRadians(180));

    private final Pose gateControl       = new Pose(44,   65.23);
    //    private final Pose gatePoseRotated   = new Pose(12.5, 58,    Math.toRadians(165)).mirror();
//    private final Pose gateRotateControl = new Pose(15.8, 61.7).mirror();
//    private final Pose gatePose          = new Pose(17,   65.9,  Math.toRadians(180)).mirror();
    private final Pose gatePoseRotated   = new Pose(12.5, 58,    Math.toRadians(165));
    private final Pose gateRotateControl = new Pose(15.8, 61.7);
    private final Pose gatePose          = new Pose(17.2,   65.9,  Math.toRadians(180));

    private final Pose parkPose = new Pose(63.9, 105.77, Math.toRadians(180));

    /**
     * The field goal that turret/flywheel/hood aim at.
     * Mirrored from blue (5, 140) to the red side.
     */
    private final Pose goalPose = new Pose(5, 140);

    // ═══════════════════════════════════════════════════════════════════════════════
    // OpMode lifecycle
    // ═══════════════════════════════════════════════════════════════════════════════

    @Override
    public void init() {
        pathTimer   = new Timer();
        opmodeTimer = new Timer();
        shakeTimer  = new Timer();
        shootTimer  = new Timer();


        toggleMotor = hardwareMap.get(DcMotor.class, "intake");
        servo       = hardwareMap.get(Servo.class,   "servo");


        shooterR = hardwareMap.get(DcMotor.class, "shooterR");
        shooterL = hardwareMap.get(DcMotor.class, "shooterL");
        shooterR.setDirection(DcMotorSimple.Direction.FORWARD);
        shooterL.setDirection(DcMotorSimple.Direction.REVERSE);

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        // Shooter encapsulates shooterR, shooterL, turretMotor, hoodServo.
        // It uses the follower pose for live turret/flywheel/hood targeting.
        shooter = new Shooter(hardwareMap, follower, telemetry);
        shooter.resetTurretEncoder();

        buildPaths();
    }

    @Override
    public void init_loop() {}

    @Override
    public void start() {
        opmodeTimer.resetTimer();
//        GoBildaPinpointDriver pinpoint=hardwareMap.get(GoBildaPinpointDriver.class,"pinpoint");
//        pinpoint.resetPosAndIMU();
        setPathState(STATE_INIT);

    }

    @Override
    public void loop() {
        follower.update();


        // ── Update targets from current robot position to goal ────────────────
        // calculateTargets sets turretTargetTicks, targetFlywheelTPS, and
        // targetHoodServoPos all in one call using the distance-based curves
        // defined in Shooter.flywheelSpeed() and Shooter.hoodAngle().
        shooter.calculateTargets(goalPose);
        double shooterPower = updateShooterVelocityPID();
        shooterR.setPower(shooterPower);
        shooterL.setPower(shooterPower);

        // ── Drive all three shooter axes with the freshly computed targets ─────
        shooter.driveTurret();
//        shooter.driveFlywheels(true);   // flywheel always spinning during auto
        shooter.driveHood();


        autonomousPathUpdate();

        telemetry.addData("Path State",      pathState);
        telemetry.addData("X",               follower.getPose().getX());
        telemetry.addData("Y",               follower.getPose().getY());
        telemetry.addData("Heading",         Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addData("ShootingStarted", shootingStarted);
        telemetry.update();
    }

    @Override
    public void stop() {}

    // ═══════════════════════════════════════════════════════════════════════════════
    // Path building
    // ═══════════════════════════════════════════════════════════════════════════════

    private void buildPaths() {
        gateRotate = follower.pathBuilder()
                .addPath(new BezierCurve(gatePose, gateRotateControl, gatePoseRotated))
                .setLinearHeadingInterpolation(gatePose.getHeading(), gatePoseRotated.getHeading())
                .build();

        scoreFirst = follower.pathBuilder()
                .addPath(new BezierCurve(startPose, control1, scorePose))
                .setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading())
                .build();

        pickupFirst = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose, control2, pickup2Pose2))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup2Pose2.getHeading())
                .build();

        score2 = follower.pathBuilder()
                .addPath(new BezierLine(pickup2Pose2, scorePose))
                .setLinearHeadingInterpolation(pickup2Pose2.getHeading(), scorePose.getHeading())
                .build();

        pickupGate = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose, gateControl, gatePose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), gatePose.getHeading())
                .build();

        scoreGate = follower.pathBuilder()
                .addPath(new BezierCurve(gatePoseRotated, gateControl, scorePose))
                .setLinearHeadingInterpolation(gatePose.getHeading(), scorePose.getHeading())
                .build();

        pickupSecond = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose, control3, pickup3Pose2))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup3Pose2.getHeading())
                .build();

        score3 = follower.pathBuilder()
                .addPath(new BezierLine(pickup3Pose2, parkPose))
                .setLinearHeadingInterpolation(pickup3Pose2.getHeading(), parkPose.getHeading())
                .build();
    }

    // ═══════════════════════════════════════════════════════════════════════════════
    // Shoot-while-moving helpers
    // ═══════════════════════════════════════════════════════════════════════════════

    /**
     * Arms the shoot sequence once the robot is within SHOOT_TRIGGER_DIST of
     * the target pose.
     */
    private void startShootIfClose(Pose target) {
        if (shootingStarted) return;
        Pose   cur = follower.getPose();
        double dx  = cur.getX() - target.getX();
        double dy  = cur.getY() - target.getY();
        if (Math.sqrt(dx * dx + dy * dy) < SHOOT_TRIGGER_DIST) {
            shootTimer.resetTimer();
            shootingStarted = true;
        }
    }

    /**
     * Runs the servo sequence after shootingStarted is true.
     * Transitions to nextState when finished.
     *
     * @return true once the sequence completes and state has advanced.
     */
    private void shoot(Timer timer, int nextState) {
        if (timer.getElapsedTimeSeconds() < 0.8) {
            servo.setPosition(SERVO_SHOOT);
        }
        else if(timer.getElapsedTimeSeconds() < 1)
        {
            servo.setPosition(SERVO_INTAKE);
        }
        else if(timer.getElapsedTimeSeconds() < 1.5) {
            servo.setPosition(SERVO_SHOOT);
        }
        else
        {
            servo.setPosition(SERVO_INTAKE);
            setPathState(nextState);
        }
    }

    // ═══════════════════════════════════════════════════════════════════════════════
    // Main state machine
    // ═══════════════════════════════════════════════════════════════════════════════

    public void autonomousPathUpdate() {
        switch (pathState) {
            case STATE_INIT:
                // Spin up shooters
                toggleMotor.setPower(1);
                setPathState(STATE_MOVE_TO_SCORE_FIRST);
                break;

            case STATE_MOVE_TO_SCORE_FIRST:
                if (!pathStarted) {
                    follower.followPath(scoreFirst, true);
                    pathStarted = true;
                }
                if (!follower.isBusy()) {
                    setPathState(STATE_SHOOT_FIRST);
                }
                break;

            case STATE_SHOOT_FIRST:
                shoot(pathTimer, STATE_MOVE_TO_PICKUP2);
                break;

            case STATE_MOVE_TO_PICKUP2:
                if (!pathStarted) {
                    follower.followPath(pickupFirst);
                    pathStarted = true;
                }
                if (!follower.isBusy()) {
                    setPathState(STATE_MOVE_TO_SCORE2);
                }
                break;

            case STATE_MOVE_TO_SCORE2:
                if (!pathStarted) {
                    follower.followPath(score2, true);
                    pathStarted = true;
                }
                if (!follower.isBusy()) {
                    setPathState(STATE_SHOOT_SECOND);
                }
                break;

            case STATE_SHOOT_SECOND:
                shoot(pathTimer, STATE_MOVE_TO_GATE);
                break;

            case STATE_MOVE_TO_GATE:
//                if (pathTimer.getElapsedTimeSeconds() < 0.05) {
//                    follower.followPath(scoreGate);
//                }
//                // Transition as soon as the robot actually arrives
//                if (!follower.isBusy()) {
//                    setPathState(STATE_SHOOT_GATE);
//                }
//                if (!pathStarted) {
//                    follower.followPath(pickupGate);
//                    pathStarted = true;
//                }
//                if (pathTimer.getElapsedTimeSeconds() > 2.8) {
//                    setPathState(STATE_MOVE_TO_SCORE_GATE);
//                }
                doGatePickup(STATE_MOVE_TO_SCORE_GATE);

                break;

            case STATE_MOVE_TO_SCORE_GATE:
                if (!pathStarted) {
                    follower.followPath(scoreGate, true);
                    pathStarted = true;
                }
                if (!follower.isBusy()) {
                    setPathState(STATE_SHOOT_GATE);
                }
                break;

            case STATE_SHOOT_GATE:
                shoot(pathTimer, STATE_MOVE_TO_GATE2);
                break;

            case STATE_MOVE_TO_GATE2:
                doGatePickup(STATE_MOVE_TO_SCORE_GATE2);
//                if (pathTimer.getElapsedTimeSeconds() < 0.05) {
//                    follower.followPath(scoreGate);
//                }
//                // Transition as soon as the robot actually arrives
//                if (!follower.isBusy()) {
//                    setPathState(STATE_SHOOT_GATE2);
//                }
                break;

            case STATE_MOVE_TO_SCORE_GATE2:
                if (!pathStarted) {
                    follower.followPath(scoreGate, true);
                    pathStarted = true;
                }
                if (!follower.isBusy()) {
                    setPathState(STATE_SHOOT_GATE2);
                }
                break;

            case STATE_SHOOT_GATE2:
                shoot(pathTimer, STATE_MOVE_TO_GATE3);
                break;

            case STATE_MOVE_TO_GATE3:
                doGatePickup(STATE_MOVE_TO_SCORE_GATE3);

//                if (!pathStarted) {
//                    follower.followPath(pickupGate);
//                    pathStarted = true;
//                }
//                if (pathTimer.getElapsedTimeSeconds() > 2.8) {
//                    setPathState(STATE_MOVE_TO_SCORE_GATE3);
//                }
//                if (pathTimer.getElapsedTimeSeconds() < 0.05) {
//                    follower.followPath(scoreGate);
//                }
//                // Transition as soon as the robot actually arrives
//                if (!follower.isBusy()) {
//                    setPathState(STATE_SHOOT_GATE3);
//                }
                break;

            case STATE_MOVE_TO_SCORE_GATE3:
                if (!pathStarted) {
                    follower.followPath(scoreGate, true);
                    pathStarted = true;
                }
                if (!follower.isBusy()) {
                    setPathState(STATE_SHOOT_GATE3);
                }
                break;

            case STATE_SHOOT_GATE3:
                shoot(pathTimer, STATE_MOVE_TO_PICKUP1);
                break;

            case STATE_MOVE_TO_PICKUP1:
                if (!pathStarted) {
                    follower.followPath(pickupSecond);
                    pathStarted = true;
                }
                if (!follower.isBusy()) {
                    setPathState(STATE_MOVE_TO_SCORE1);
                }
                break;

            case STATE_MOVE_TO_SCORE1:
                if (!pathStarted) {
                    follower.followPath(score3, true);
                    pathStarted = true;
                }
                if (!follower.isBusy()) {
                    setPathState(STATE_SHOOT_FIRST_PICKUP);
                }
                break;
//                if (!pathStarted) {
//                    follower.followPath(score3, true); // ← fixed: was scoreGate
//                    pathStarted = true;
//                }
//                if (!follower.isBusy()) {
//                    setPathState(STATE_SHOOT_FIRST_PICKUP);
//                }
//                break;

            case STATE_SHOOT_FIRST_PICKUP:
                shoot(pathTimer, STATE_DONE);
                break;

//            case STATE_PARK:
//                if (!pathStarted) {
//                    follower.followPath(park);
//                    pathStarted = true;
//                }
//                if (pathTimer.getElapsedTimeSeconds() > 0.5) {
//                    setPathState(STATE_DONE);
//                }
//                break;
        }
    }

    private void setPathState(int pState) {
        pathState       = pState;
        pathTimer.resetTimer();
        pathStarted     = false;
        shootingStarted = false;
    }

    // ═══════════════════════════════════════════════════════════════════════════════
    // Gate pickup
    // ═══════════════════════════════════════════════════════════════════════════════

    private void doGatePickup(int nextState) {
        if (!atGate) {
            if (!pathStarted) {
                follower.followPath(pickupGate);
                pathStarted = true;
            }
            if (!follower.isBusy()) {
                atGate      = true;
                pathStarted = false;
                shakeTimer.resetTimer();
            }
            return;
        }

        if (!pathStarted) {
            follower.followPath(gateRotate);
            pathStarted = true;
        }
        if (!follower.isBusy() && shakeTimer.getElapsedTimeSeconds() > 1.6) {
            atGate = false;
            setPathState(nextState);
        }
    }

    private double updateShooterVelocityPID() {
        final double targetTps = 1450;

        long   nowNs = System.nanoTime();
        double dtSec = (nowNs - lastTimeNs) / 1_000_000_000.0;

        if (dtSec <= 0.001) {
            lastTimeNs     = nowNs;
            lastEncoderPos = shooterR.getCurrentPosition();
            return 0.0;
        }

        int    currentPos  = shooterR.getCurrentPosition();
        double velocityTPS = (currentPos - lastEncoderPos) / dtSec;
        double error       = targetTps - velocityTPS;

        double pTerm = RobotConstants.FLYWHEEL_KP * error;

        pidIntegral += error * dtSec;
        double iTerm = clamp(RobotConstants.FLYWHEEL_KI * pidIntegral,
                -RobotConstants.SHOOTER_I_MAX, RobotConstants.SHOOTER_I_MAX);

        double dTerm = RobotConstants.FLYWHEEL_KD * (error - pidLastError) / dtSec;

        double power = clamp(pTerm + iTerm + dTerm, 0.0, 1.0);

        pidLastError   = error;
        lastEncoderPos = currentPos;
        lastTimeNs     = nowNs;

        telemetry.addData("ShooterTPS", "%.1f", velocityTPS);
        telemetry.addData("ShooterErr", "%.1f", error);

        return power;
    }

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }
}