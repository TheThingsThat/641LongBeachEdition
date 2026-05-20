package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.configs.RobotConstants;
import org.firstinspires.ftc.teamcode.configs.Shooter;
import org.firstinspires.ftc.teamcode.configs.pedroPathing.Constants;

@TeleOp(name = "BLUE TELEOP")
public class BlueTeleop extends OpMode {

    // ═══════════════════════════════════════════════════════════════════════════
    // ROBOT MODE
    // ═══════════════════════════════════════════════════════════════════════════

    /**
     * AUTO        — Turret, flywheel, and hood all driven by distance polynomials.
     * TURRET_ONLY — Turret auto-tracks; flywheel and hood at fixed constants.
     * FULL_MANUAL — Turret, flywheel, and hood all at fixed constants.
     * ADJUSTABLE  — Turret auto-tracks; flywheel and hood adjustable live via gamepad2.
     */
    private enum RobotMode { AUTO, TURRET_ONLY, FULL_MANUAL, ADJUSTABLE }
    private RobotMode mode = RobotMode.AUTO; // default to AUTO at match start

    // ─── Gamepad2 mode-switch edge detectors ─────────────────────────────────
    private boolean lastGp2Y  = false;
    private boolean lastGp2B  = false;
    private boolean lastGp2A  = false;
    private boolean lastGp2X  = false;
    private boolean lastGp2LB = false; // shooter on/off toggle
    private boolean lastGp1B = false;

    // ═══════════════════════════════════════════════════════════════════════════
    // PEDRO
    // ═══════════════════════════════════════════════════════════════════════════

    private Follower follower;
    private final Pose startPose = RobotConstants.startPose;

    private final Pose goalPose = RobotConstants.blueGoal;

    // ═══════════════════════════════════════════════════════════════════════════
    // HARDWARE
    // ═══════════════════════════════════════════════════════════════════════════

    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private DcMotor toggleMotor;
    private Shooter shooter;
    private Servo   kickerServo;

    // ─── Laser sensor & RGB indicator ────────────────────────────────────────
    private DigitalChannel laserInput;
    private Servo          rgbLight;

    // goBILDA RGB Indicator Light color positions (Product Insight #4)
    private static final double RGB_RED    = 0.277; // 1100 μs — no item
    private static final double RGB_YELLOW = 0.388; // 1300 μs — 1st pass
    private static final double RGB_BLUE   = 0.611; // 1700 μs — 2nd pass
    private static final double RGB_GREEN  = 0.500; // 1500 μs — 3rd pass

    // ═══════════════════════════════════════════════════════════════════════════
    // STATE
    // ═══════════════════════════════════════════════════════════════════════════

    // ─── Laser / pass counting ────────────────────────────────────────────────
    private int     passCount    = 0;
    private boolean lastDetected = false; // tracks active-low beam state

    // ─── Gamepad1 edge detectors ──────────────────────────────────────────────
    private boolean lastGp1A  = false; // intake toggle

    // ─── Shooter ─────────────────────────────────────────────────────────────
    private boolean shooterOn = true;

    // ─── Intake ───────────────────────────────────────────────────────────────
    private boolean intakeOn = true;

    // ─── Kicker triple-kick sequencer ────────────────────────────────────────
    // The trigger fires three down/up cycles then rests in the DOWN position.
    // Sequence: down→up→down→up→down→up→down (6 transitions, final = DOWN).
    private boolean        kickerSeqActive  = false;
    private int            kickerSeqStep    = 0;    // 0..5
    private final ElapsedTime kickerSeqTimer = new ElapsedTime();
    private boolean        lastGp1RT        = false; // right-trigger edge

    // ─── ADJUSTABLE mode live values ─────────────────────────────────────────
    // Initialised from the fixed constants; adjusted by gamepad2 d-pad.
    private double adjustableHoodPos    = RobotConstants.fixedHoodPos;
    private double adjustableFlywheelTPS = RobotConstants.fixedFlywheelTPS;

    // Gamepad2 d-pad edge detectors for ADJUSTABLE mode
    private boolean lastGp2DUp   = false;
    private boolean lastGp2DDown = false;
    private boolean lastGp2DLeft = false;
    private boolean lastGp2DRight = false;

    private final ElapsedTime loopTimer = new ElapsedTime();

    // ═══════════════════════════════════════════════════════════════════════════
    // INIT
    // ═══════════════════════════════════════════════════════════════════════════

    @Override
    public void init() {
        // Drive motors
        frontLeft  = hardwareMap.get(DcMotor.class, "fl");
        frontRight = hardwareMap.get(DcMotor.class, "fr");
        backLeft   = hardwareMap.get(DcMotor.class, "bl");
        backRight  = hardwareMap.get(DcMotor.class, "br");

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Kicker servo
        kickerServo = hardwareMap.get(Servo.class, "servo");

        // Intake motor
        toggleMotor = hardwareMap.get(DcMotor.class, "intake");
        toggleMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        toggleMotor.setPower(0);

        // Pedro follower — must be created before Shooter
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        follower.update();
        follower.startTeleopDrive();

        // Shooter subsystem
        shooter = new Shooter(hardwareMap, follower, telemetry);
        shooter.resetTurretEncoder();

        // ── Seed the turret toward the goal from the known starting pose ──────
        // Without this, turretTargetTicks = 0 and the turret slews to robot-forward
        // on the first loop tick before calculateTargets() runs.
        shooter.calculateTurretTarget(goalPose.getX(), goalPose.getY());

        // Laser sensor & RGB light
        laserInput = hardwareMap.get(DigitalChannel.class, "laserDigitalInput");
        laserInput.setMode(DigitalChannel.Mode.INPUT);
        rgbLight = hardwareMap.get(Servo.class, "rgbLight");
        rgbLight.setPosition(RGB_RED);

        loopTimer.reset();

        telemetry.addLine("Initialized — Mode: AUTO");
        telemetry.update();
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // LOOP
    // ═══════════════════════════════════════════════════════════════════════════

    @Override
    public void loop() {
        follower.update();

        // ── Drive (gamepad1 sticks) ────────────────────────────────────────
        follower.setTeleOpDrive(
                gamepad1.left_stick_y,
                gamepad1.left_stick_x,
                -gamepad1.right_stick_x,
                false
        );

        // ── Mode switching (gamepad2 face buttons, rising-edge) ────────────
        handleModeSwitch();

        // ── Shooter enable toggle (gamepad2 LB, rising-edge) ──────────────
        boolean gp2LB = gamepad2.left_bumper;
        if (gp2LB && !lastGp2LB) shooterOn = !shooterOn;
        lastGp2LB = gp2LB;

        // ── Shooter subsystem (turret + flywheel + hood) ──────────────────
        handleShooterMode();

        // ── Gamepad1 binds (active in all modes) ──────────────────────────
        handleGamepad1();

        // ── Gamepad2 binds (only active in ADJUSTABLE mode) ───────────────
        if (mode == RobotMode.ADJUSTABLE) handleGamepad2Adjustable();

        // ── Kicker triple-kick sequencer ──────────────────────────────────
        handleKickerSequencer();

        // ── Laser sensor & RGB indicator ──────────────────────────────────
        handleLaserAndRgb();

        // ── Pinpoint Reset ──────────────────────────────────
        handlePoseReset();

        // ── Telemetry ──────────────────────────────────────────────────────
        telemetry.addData("Mode",          mode.name());
        telemetry.addData("ShooterOn",     shooterOn);
        telemetry.addData("TPS Target",    "%.0f", shooter.targetFlywheelTPS);
        telemetry.addData("TPS Actual", "%.1f", shooter.lastFlywheelTPS);
        telemetry.addData("Hood Target",   "%.3f", shooter.targetHoodServoPos);
        telemetry.addData("Hood Actual",     "%.3f", shooter.hoodServo.getPosition());
        telemetry.addData("Robot Pose",    follower.getPose());
        telemetry.addData("Dist from Goal", "%.1f", shooter.getRobotDistanceToGoal(goalPose));
        telemetry.addData("Pass Count",    passCount);
        telemetry.update();
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // MODE SWITCH — gamepad2 face buttons
    // ═══════════════════════════════════════════════════════════════════════════

    private void handleModeSwitch() {
        boolean gp2Y = gamepad2.y;
        boolean gp2B = gamepad2.b;
        boolean gp2A = gamepad2.a;
        boolean gp2X = gamepad2.x;

        if (gp2Y && !lastGp2Y) mode = RobotMode.AUTO;
        if (gp2B && !lastGp2B) mode = RobotMode.TURRET_ONLY;
        if (gp2A && !lastGp2A) {
            mode = RobotMode.FULL_MANUAL;
            shooter.turretTargetTicks = (int) (-RobotConstants.lockedTurretDeg * RobotConstants.TICKS_PER_DEGREE);
        }
        if (gp2X && !lastGp2X) {
            // Reset adjustable values to fixed constants when entering ADJUSTABLE
            if (mode != RobotMode.ADJUSTABLE) {
                adjustableHoodPos     = RobotConstants.fixedHoodPos;
                adjustableFlywheelTPS = RobotConstants.fixedFlywheelTPS;
                shooter.turretTargetTicks = (int) (RobotConstants.lockedTurretDeg * RobotConstants.TICKS_PER_DEGREE);
            }
            mode = RobotMode.ADJUSTABLE;
        }

        lastGp2Y = gp2Y;
        lastGp2B = gp2B;
        lastGp2A = gp2A;
        lastGp2X = gp2X;
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // SHOOTER MODE DISPATCH
    // ═══════════════════════════════════════════════════════════════════════════

    private void handleShooterMode() {
        switch (mode) {

            case AUTO:
                // All three axes driven by distance polynomial
                shooter.calculateTargets(goalPose);
                shooter.driveTurret();
                shooter.driveFlywheels(shooterOn);
                shooter.driveHood();
                break;

            case TURRET_ONLY:
                // Turret auto-tracks; flywheel and hood fixed
                shooter.calculateTurretTarget(goalPose.getX(), goalPose.getY());
                shooter.targetFlywheelTPS  = RobotConstants.fixedFlywheelTPS;
                shooter.targetHoodServoPos = RobotConstants.fixedHoodPos;
                shooter.driveTurret();
                shooter.driveFlywheels(shooterOn);
                shooter.driveHood();
                break;

            case FULL_MANUAL:
                // All three axes fixed — no auto tracking at all
                shooter.targetFlywheelTPS  = RobotConstants.fixedFlywheelTPS;
                shooter.targetHoodServoPos = RobotConstants.fixedHoodPos;
                shooter.driveTurret();
                shooter.driveFlywheels(shooterOn);
                shooter.driveHood();
                break;

            case ADJUSTABLE:
                // Turret auto-tracks; flywheel and hood driven from live adjustable values
                shooter.calculateTurretTarget(goalPose.getX(), goalPose.getY());
                shooter.targetFlywheelTPS  = adjustableFlywheelTPS;
                shooter.targetHoodServoPos = adjustableHoodPos;
                shooter.driveTurret();
                shooter.driveFlywheels(shooterOn);
                shooter.driveHood();
                break;
        }
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // GAMEPAD1 — active in ALL modes
    //
    //   A             — toggle intake on/off
    //   X (held)      — run intake in reverse
    //   LB            — set kicker servo to DOWN position
    //   RB            — raise kicker servo (up)
    //   B             — kicker DOWN (manual)
    //   Right trigger — triple-kick sequence (ends in DOWN position)
    // ═══════════════════════════════════════════════════════════════════════════

    private void handleGamepad1() {
        // ── Intake toggle (A, rising edge) ────────────────────────────────
        boolean gp1A = gamepad1.a;
        if (gp1A && !lastGp1A) intakeOn = !intakeOn;
        lastGp1A = gp1A;

        // ── Intake reverse (X, held) — overrides toggle while held ────────
        if (gamepad1.x) {
            toggleMotor.setPower(-1.0);
        } else {
            toggleMotor.setPower(intakeOn ? 1.0 : 0.0);
        }

        // ── Kicker manual positions ────────────────────────────────────────
        // LB → down, right_bumper → up, b → down
        // (triple-kick via right_trigger is handled in handleKickerSequencer)
        if (!kickerSeqActive) {
            if (gamepad1.left_bumper)  kickerServo.setPosition(RobotConstants.kickerDownPos);
            if (gamepad1.right_bumper) kickerServo.setPosition(RobotConstants.kickerUpPos);
        }

        // ── RGB / pass-count reset ─────────────────────────────────────────
        // Resets on rising edge of right_bumper OR right_trigger
        boolean resetNow = (gamepad1.right_bumper && !lastGp1RB)
                || (gamepad1.right_trigger > 0.5 && !lastGp1RT);
        if (resetNow) passCount = 0;
        lastGp1RB = gamepad1.right_bumper;
        // lastGp1RT is updated inside handleKickerSequencer to avoid double-edge
    }

    // Edge detector for right_bumper used in handleGamepad1 (needs field scope)
    private boolean lastGp1RB = false;

    // ═══════════════════════════════════════════════════════════════════════════
    // KICKER TRIPLE-KICK SEQUENCER
    //
    // right_trigger rising edge starts a 6-step sequence:
    //   step 0: DOWN
    //   step 1: UP
    //   step 2: DOWN
    //   step 3: UP
    //   step 4: DOWN
    //   step 5: UP
    // After step 5 the sequence completes and the kicker rests in the DOWN position.
    // Each step waits kickerStaggerSec before advancing.
    // ═══════════════════════════════════════════════════════════════════════════

    private void handleKickerSequencer() {
        boolean gp1RT = gamepad1.right_trigger > 0.5;

        // Start sequence on rising edge
        if (gp1RT && !lastGp1RT && !kickerSeqActive) {
            kickerSeqActive = true;
            kickerSeqStep   = 0;
            kickerSeqTimer.reset();
            kickerServo.setPosition(RobotConstants.kickerDownPos); // step 0
        }
        lastGp1RT = gp1RT;

        if (!kickerSeqActive) return;

        // Advance through steps as each stagger interval elapses
        if (kickerSeqTimer.seconds() >= RobotConstants.kickerStaggerSec) {
            kickerSeqStep++;
            kickerSeqTimer.reset();

            if (kickerSeqStep >= 6) {
                // Sequence complete — leave kicker in DOWN position
                kickerServo.setPosition(RobotConstants.kickerDownPos);
                kickerSeqActive = false;
            } else {
                // Even steps = DOWN, odd steps = UP
                if (kickerSeqStep % 2 == 0) {
                    kickerServo.setPosition(RobotConstants.kickerDownPos);
                } else {
                    kickerServo.setPosition(RobotConstants.kickerUpPos);
                }
            }
        }
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // GAMEPAD2 ADJUSTABLE MODE CONTROLS
    // Only called when mode == ADJUSTABLE.
    //
    //   D-pad up/down   — hood servo position (step size: adjustableHoodStep)
    //   D-pad left/right— flywheel TPS (step size: adjustableFlywheelStep)
    //                     left = decrease, right = increase
    // ═══════════════════════════════════════════════════════════════════════════

    private void handleGamepad2Adjustable() {
        boolean dUp    = gamepad2.dpad_up;
        boolean dDown  = gamepad2.dpad_down;
        boolean dLeft  = gamepad2.dpad_left;
        boolean dRight = gamepad2.dpad_right;

        if (dUp    && !lastGp2DUp)    adjustableHoodPos     += RobotConstants.adjustableHoodStep;
        if (dDown  && !lastGp2DDown)  adjustableHoodPos     -= RobotConstants.adjustableHoodStep;
        if (dRight && !lastGp2DRight) adjustableFlywheelTPS += RobotConstants.adjustableFlywheelStep;
        if (dLeft  && !lastGp2DLeft)  adjustableFlywheelTPS -= RobotConstants.adjustableFlywheelStep;

        // Clamp to legal ranges
        adjustableHoodPos     = clamp(adjustableHoodPos, RobotConstants.hoodServoMin, RobotConstants.hoodServoMax);
        adjustableFlywheelTPS = clamp(adjustableFlywheelTPS, 0.0,  6000.0);

        lastGp2DUp    = dUp;
        lastGp2DDown  = dDown;
        lastGp2DLeft  = dLeft;
        lastGp2DRight = dRight;

        telemetry.addData("Adj Hood",    "%.3f", adjustableHoodPos);
        telemetry.addData("Adj TPS",     "%.0f", adjustableFlywheelTPS);
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // LASER SENSOR & RGB INDICATOR
    //
    // Active-low sensor: getState() == false means beam is BROKEN (object present).
    // Pass count increments on the rising edge of beam-broken.
    // Capped at 3.  Reset by right_bumper or right_trigger (handled in handleGamepad1).
    // ═══════════════════════════════════════════════════════════════════════════

    private void handleLaserAndRgb() {
        boolean beamBroken = !laserInput.getState();

        if (beamBroken && !lastDetected) {
            passCount++;
            if (passCount > 3) passCount = 3;
        }
        lastDetected = beamBroken;

        if      (!beamBroken && passCount == 0) rgbLight.setPosition(RGB_RED);
        else if (passCount == 1)                rgbLight.setPosition(RGB_YELLOW);
        else if (passCount == 2)                rgbLight.setPosition(RGB_BLUE);
        else if (passCount >= 3)                rgbLight.setPosition(RGB_GREEN);
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // PINPOINT RESET
    //
    // ROBOT WILL PARK IN THE HUMAN PLAYER ZONE.
    // PRESSING B ON GAMEPAD1 WILL SET THE ROBOT POSITION TO BLUE/RED RESET POSE
    // ═══════════════════════════════════════════════════════════════════════════
    private void handlePoseReset() {
        boolean gp1B = gamepad1.b;
        if (gp1B && !lastGp1B) {
            follower.setPose(RobotConstants.blueReset);
        }
        lastGp1B = gp1B;
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // HELPERS
    // ═══════════════════════════════════════════════════════════════════════════

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }
}