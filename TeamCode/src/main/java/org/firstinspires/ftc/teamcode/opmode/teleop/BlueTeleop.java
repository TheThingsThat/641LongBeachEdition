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
     * AUTO        — turret, flywheel, hood all driven by distance polynomials.
     * TURRET_ONLY — turret auto-tracks; flywheel and hood at fixed constants.
     * FULL_MANUAL — turret, flywheel, hood all at fixed constants.
     * ADJUSTABLE  — turret auto-tracks; flywheel and hood adjustable via gamepad2.
     */
    private enum RobotMode { AUTO, TURRET_ONLY, FULL_MANUAL, ADJUSTABLE }
    private RobotMode mode = RobotMode.AUTO;

    // ─── Gamepad2 mode-switch edge detectors ─────────────────────────────────
    private boolean lastGp2Y  = false;
    private boolean lastGp2B  = false;
    private boolean lastGp2A  = false;
    private boolean lastGp2X  = false;
    private boolean lastGp2LB = false;
    private boolean lastGp1B  = false;

    // ═══════════════════════════════════════════════════════════════════════════
    // PEDRO
    // ═══════════════════════════════════════════════════════════════════════════

    private Follower follower;
    private final Pose startPose = RobotConstants.startPose;
    private final Pose goalPose  = RobotConstants.blueGoal;

    // ═══════════════════════════════════════════════════════════════════════════
    // HARDWARE
    // ═══════════════════════════════════════════════════════════════════════════

    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private DcMotor toggleMotor;
    private Shooter shooter;
    private Servo   kickerServo;

    private DigitalChannel laserInput;
    private Servo          rgbLight;

    // goBILDA RGB Indicator Light positions
    private static final double RGB_RED    = 0.277; // 1100 μs — no item
    private static final double RGB_YELLOW = 0.388; // 1300 μs — 1st pass
    private static final double RGB_BLUE   = 0.611; // 1700 μs — 2nd pass
    private static final double RGB_GREEN  = 0.500; // 1500 μs — 3rd pass

    // ═══════════════════════════════════════════════════════════════════════════
    // STATE
    // ═══════════════════════════════════════════════════════════════════════════

    private int     passCount    = 0;
    private boolean lastDetected = false; // active-low beam state

    private boolean lastGp1A  = false; // intake toggle
    private boolean lastGp1RB = false; // pass-count reset edge

    private boolean shooterOn = true;
    private boolean intakeOn  = true;

    // Kicker triple-kick sequencer: 6 transitions ending in DOWN.
    private boolean           kickerSeqActive = false;
    private int               kickerSeqStep   = 0;
    private final ElapsedTime kickerSeqTimer  = new ElapsedTime();
    private boolean           lastGp1RT       = false;

    // Kicker single-press (RB): UP, then DOWN after kickerSinglePressSec.
    private boolean           kickerSingleActive = false;
    private final ElapsedTime kickerSingleTimer  = new ElapsedTime();

    // ADJUSTABLE mode live values (seeded from fixed constants).
    private double adjustableHoodPos     = RobotConstants.fixedHoodPos;
    private double adjustableFlywheelTPS = RobotConstants.fixedFlywheelTPS;

    private boolean lastGp2DUp    = false;
    private boolean lastGp2DDown  = false;
    private boolean lastGp2DLeft  = false;
    private boolean lastGp2DRight = false;

    // ═══════════════════════════════════════════════════════════════════════════
    // INIT
    // ═══════════════════════════════════════════════════════════════════════════

    @Override
    public void init() {
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

        kickerServo = hardwareMap.get(Servo.class, "servo");

        toggleMotor = hardwareMap.get(DcMotor.class, "intake");
        toggleMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        toggleMotor.setPower(0);

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        follower.update();
        follower.startTeleopDrive();

        shooter = new Shooter(hardwareMap, follower, telemetry);
        shooter.resetTurretEncoder();

        // Seed turret toward goal so it doesn't slew to robot-forward on tick 1.
        shooter.calculateTurretTarget(goalPose.getX(), goalPose.getY());

        laserInput = hardwareMap.get(DigitalChannel.class, "laserDigitalInput");
        laserInput.setMode(DigitalChannel.Mode.INPUT);
        rgbLight = hardwareMap.get(Servo.class, "rgbLight");
        rgbLight.setPosition(RGB_RED);

        telemetry.addLine("Initialized — Mode: AUTO");
        telemetry.update();
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // LOOP
    // ═══════════════════════════════════════════════════════════════════════════

    @Override
    public void loop() {
        follower.update();

        follower.setTeleOpDrive(
                gamepad1.left_stick_y,
                gamepad1.left_stick_x,
                -gamepad1.right_stick_x,
                false
        );

        handleModeSwitch();

        boolean gp2LB = gamepad2.left_bumper;
        if (gp2LB && !lastGp2LB) shooterOn = !shooterOn;
        lastGp2LB = gp2LB;

        handleShooterMode();
        handleGamepad1();
        if      (mode == RobotMode.ADJUSTABLE) handleGamepad2Adjustable();
        else if (mode == RobotMode.AUTO)       handleGamepad2AutoOffsets();
        handleKickerSequencer();
        handleKickerSinglePress();
        handleLaserAndRgb();
        handlePoseReset();

        telemetry.addData("Mode",           mode.name());
        telemetry.addData("ShooterOn",      shooterOn);
        telemetry.addData("TPS Target",     "%.0f", shooter.targetFlywheelTPS);
        telemetry.addData("TPS Actual",     "%.1f", shooter.lastFlywheelTPS);
        telemetry.addData("Hood Target",    "%.3f", shooter.targetHoodServoPos);
        telemetry.addData("Hood Actual",    "%.3f", shooter.hoodServo.getPosition());
        telemetry.addData("Robot Pose",     follower.getPose());
        telemetry.addData("Dist from Goal", "%.1f", shooter.getRobotDistanceToGoal(goalPose));
        telemetry.addData("Pass Count",     passCount);
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
                shooter.calculateTargets(goalPose);
                break;

            case TURRET_ONLY:
                shooter.calculateTurretTarget(goalPose.getX(), goalPose.getY());
                shooter.targetFlywheelTPS  = RobotConstants.fixedFlywheelTPS;
                shooter.targetHoodServoPos = RobotConstants.fixedHoodPos;
                break;

            case FULL_MANUAL:
                shooter.targetFlywheelTPS  = RobotConstants.fixedFlywheelTPS;
                shooter.targetHoodServoPos = RobotConstants.fixedHoodPos;
                break;

            case ADJUSTABLE:
                shooter.calculateTurretTarget(goalPose.getX(), goalPose.getY());
                shooter.targetFlywheelTPS  = adjustableFlywheelTPS;
                shooter.targetHoodServoPos = adjustableHoodPos;
                break;
        }

        shooter.driveTurret();
        shooter.driveFlywheels(shooterOn);
        shooter.driveHood();
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // GAMEPAD1 — active in ALL modes
    //   A             — toggle intake on/off
    //   LB (held)     — run intake in reverse
    //   RB            — single-press kick: UP then DOWN after kickerSinglePressSec
    //   Right trigger — triple-kick sequence (ends in DOWN)
    // ═══════════════════════════════════════════════════════════════════════════

    private void handleGamepad1() {
        boolean gp1A = gamepad1.a;
        if (gp1A && !lastGp1A) intakeOn = !intakeOn;
        lastGp1A = gp1A;

        if (gamepad1.left_bumper) {
            toggleMotor.setPower(-1.0);
        } else {
            toggleMotor.setPower(intakeOn ? 1.0 : 0.0);
        }

        // Pass-count reset: RB rising edge OR RT rising edge.
        // lastGp1RB is updated in handleKickerSinglePress; lastGp1RT in handleKickerSequencer.
        boolean resetNow = (gamepad1.right_bumper && !lastGp1RB)
                || (gamepad1.right_trigger > 0.5 && !lastGp1RT);
        if (resetNow) passCount = 0;
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // KICKER TRIPLE-KICK SEQUENCER
    //
    // right_trigger rising edge starts a 6-step sequence (DOWN/UP/DOWN/UP/DOWN/UP);
    // after step 5 the kicker is left in DOWN. Each step waits kickerStaggerSec.
    // ═══════════════════════════════════════════════════════════════════════════

    private void handleKickerSequencer() {
        boolean gp1RT = gamepad1.right_trigger > 0.5;

        if (gp1RT && !lastGp1RT && !kickerSeqActive) {
            kickerSeqActive = true;
            kickerSeqStep   = 0;
            kickerSeqTimer.reset();
            kickerServo.setPosition(RobotConstants.kickerDownPos);
        }
        lastGp1RT = gp1RT;

        if (!kickerSeqActive) return;

        if (kickerSeqTimer.seconds() >= RobotConstants.kickerStaggerSec) {
            kickerSeqStep++;
            kickerSeqTimer.reset();

            if (kickerSeqStep >= 6) {
                kickerServo.setPosition(RobotConstants.kickerDownPos);
                kickerSeqActive = false;
            } else {
                kickerServo.setPosition(kickerSeqStep % 2 == 0
                        ? RobotConstants.kickerDownPos
                        : RobotConstants.kickerUpPos);
            }
        }
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // KICKER SINGLE-PRESS (right bumper)
    //
    // Rising edge → kicker UP. After kickerSinglePressSec elapses → kicker DOWN.
    // Ignored while the triple-kick sequence is active.
    // ═══════════════════════════════════════════════════════════════════════════

    private void handleKickerSinglePress() {
        boolean gp1RB = gamepad1.right_bumper;

        if (gp1RB && !lastGp1RB && !kickerSeqActive && !kickerSingleActive) {
            kickerSingleActive = true;
            kickerSingleTimer.reset();
            kickerServo.setPosition(RobotConstants.kickerUpPos);
        }
        lastGp1RB = gp1RB;

        if (kickerSingleActive && kickerSingleTimer.seconds() >= RobotConstants.kickerSinglePressSec) {
            kickerServo.setPosition(RobotConstants.kickerDownPos);
            kickerSingleActive = false;
        }
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // GAMEPAD2 ADJUSTABLE MODE CONTROLS
    //   D-pad up/down    — hood servo position
    //   D-pad left/right — flywheel TPS
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

        adjustableHoodPos     = clamp(adjustableHoodPos, RobotConstants.hoodServoMin, RobotConstants.hoodServoMax);
        adjustableFlywheelTPS = clamp(adjustableFlywheelTPS, 0.0, 6000.0);

        lastGp2DUp    = dUp;
        lastGp2DDown  = dDown;
        lastGp2DLeft  = dLeft;
        lastGp2DRight = dRight;

        telemetry.addData("Adj Hood", "%.3f", adjustableHoodPos);
        telemetry.addData("Adj TPS",  "%.0f", adjustableFlywheelTPS);
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // GAMEPAD2 AUTO MODE OFFSETS
    // Only called when mode == AUTO. Trims the distance-polynomial outputs.
    //   D-pad up/down    — hoodOffset
    //   D-pad left/right — flywheelOffset (left = decrease, right = increase)
    // ═══════════════════════════════════════════════════════════════════════════

    private void handleGamepad2AutoOffsets() {
        boolean dUp    = gamepad2.dpad_up;
        boolean dDown  = gamepad2.dpad_down;
        boolean dLeft  = gamepad2.dpad_left;
        boolean dRight = gamepad2.dpad_right;

        if (dUp    && !lastGp2DUp)    RobotConstants.hoodOffset     += RobotConstants.adjustableHoodStep;
        if (dDown  && !lastGp2DDown)  RobotConstants.hoodOffset     -= RobotConstants.adjustableHoodStep;
        if (dRight && !lastGp2DRight) RobotConstants.flywheelOffset += RobotConstants.adjustableFlywheelStep;
        if (dLeft  && !lastGp2DLeft)  RobotConstants.flywheelOffset -= RobotConstants.adjustableFlywheelStep;

        lastGp2DUp    = dUp;
        lastGp2DDown  = dDown;
        lastGp2DLeft  = dLeft;
        lastGp2DRight = dRight;

        telemetry.addData("Hood Offset",     "%.3f", RobotConstants.hoodOffset);
        telemetry.addData("Flywheel Offset", "%.0f", RobotConstants.flywheelOffset);
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // LASER SENSOR & RGB INDICATOR
    // Active-low: getState() == false means beam BROKEN. Pass count caps at 3.
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
    // PINPOINT RESET — gamepad1 B snaps pose to blueReset (human player zone).
    // ═══════════════════════════════════════════════════════════════════════════

    private void handlePoseReset() {
        boolean gp1B = gamepad1.b;
        if (gp1B && !lastGp1B) follower.setPose(RobotConstants.blueReset);
        lastGp1B = gp1B;
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // HELPERS
    // ═══════════════════════════════════════════════════════════════════════════

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }
}
