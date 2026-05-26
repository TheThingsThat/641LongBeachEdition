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
     * TURRET_ONTLY — turret auto-tracks; flywheel and hood at fixed constants.
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
    private Shooter shooter; // owns the kicker servo

    private DigitalChannel laserInput;
    private Servo          rgbLight;

    // goBILDA RGB Indicator Light positions
    private static final double RGB_RED   = 0.277; // 1100 μs — no ball seated
    private static final double RGB_GREEN = 0.500; // 1500 μs — ball seated at 3rd slot

    // Intake-detect rumble thresholds
    private static final double LASER_DETECT_RUMBLE_SEC = 0.5;  // continuous break needed to rumble
    private static final double LASER_DEBOUNCE_OFF_SEC  = 0.05; // brief clears shorter than this don't reset the timer
    private static final int    RUMBLE_MS               = 300;

    // ═══════════════════════════════════════════════════════════════════════════
    // STATE
    // ═══════════════════════════════════════════════════════════════════════════

    // Debounced intake-detect state for the rumble trigger and RGB indicator.
    private boolean           debouncedBeamBroken = false;
    private boolean           rumbleFired         = false;
    private final ElapsedTime beamBrokenTimer     = new ElapsedTime();
    private final ElapsedTime beamClearTimer      = new ElapsedTime();

    private boolean lastGp1A = false; // intake toggle

    private boolean shooterOn = true;
    private boolean intakeOn  = true;

    // Gamepad1 kicker rising-edge trackers.
    private boolean lastGp1LB = false; // LB rising → single-press kick
    private boolean lastGp1RB = false; // RB rising → triple kick (TIMED)
    private boolean lastGp1RT = false; // RT rising → triple kick (FLYWHEEL-GATED)

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
        
        // Robot-centric only in FULL_MANUAL; all other modes drive field-centric.
        follower.setTeleOpDrive(
                gamepad1.left_stick_y,
                gamepad1.left_stick_x,
                -gamepad1.right_stick_x,
                mode == RobotMode.FULL_MANUAL
        );

        handleModeSwitch();

        boolean gp2LB = gamepad2.left_bumper;
        if (gp2LB && !lastGp2LB) shooterOn = !shooterOn;
        lastGp2LB = gp2LB;

        handleShooterMode();
        handleGamepad1();
        if      (mode == RobotMode.ADJUSTABLE) handleGamepad2Adjustable();
        else if (mode == RobotMode.AUTO)       handleGamepad2AutoOffsets();
        handleKickerSinglePress(); // gp1 LB
        handleKickerTriple();      // gp1 RB (timed) / RT (flywheel-gated)
        handleKickerManual();      // gp1 LT held = UP, released = DOWN (suppressed while a sequence runs)
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
    // GAMEPAD1
    //   A             — toggle intake on/off
    //   B             — pinpoint pose reset (handlePoseReset)
    //   LT (held)     — kicker UP; release returns kicker DOWN
    //   LB (rising)   — single-press kick (UP → auto DOWN after kickerSinglePressSec)
    //   RB (rising)   — triple kick, TIMED (kickerTripleCycleGapSec between cycles)
    //   RT (rising)   — triple kick, FLYWHEEL-GATED (waits for flywheelAtSpeed() each cycle)
    //
    // GAMEPAD2
    //   RB (held)     — run intake in reverse (-0.5 power)
    // ═══════════════════════════════════════════════════════════════════════════

    private void handleGamepad1() {
        boolean gp1A = gamepad1.a;
        if (gp1A && !lastGp1A) intakeOn = !intakeOn;
        lastGp1A = gp1A;

        // Intake reverse moved to gp2 RB.
        if (gamepad2.right_bumper) {
            toggleMotor.setPower(-0.5);
        } else {
            toggleMotor.setPower(intakeOn ? 1.0 : 0.0);
        }
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // KICKER SINGLE-PRESS (gp1 LB)
    // Rising edge → kicker UP. After kickerSinglePressSec elapses → kicker DOWN.
    // ═══════════════════════════════════════════════════════════════════════════

    private void handleKickerSinglePress() {
        boolean gp1LB = gamepad1.left_bumper;
        if (gp1LB && !lastGp1LB) shooter.startKick();
        lastGp1LB = gp1LB;

        shooter.updateKicker();
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // KICKER TRIPLE-KICK (gp1 RB = TIMED, gp1 RT = FLYWHEEL-GATED)
    // Each rising edge starts a 3-cycle up-down sequence. The mode flag picks
    // whether the cycle-to-cycle gap is timed or gated on flywheelAtSpeed().
    // ═══════════════════════════════════════════════════════════════════════════

    private void handleKickerTriple() {
        boolean gp1RB = gamepad1.right_bumper;
        if (gp1RB && !lastGp1RB) shooter.startTripleKick(false); // TIMED
        lastGp1RB = gp1RB;

        boolean gp1RT = gamepad1.right_trigger > 0.5;
        if (gp1RT && !lastGp1RT) shooter.startTripleKick(true);  // FLYWHEEL-GATED
        lastGp1RT = gp1RT;

        shooter.updateTripleKick();
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // KICKER MANUAL (gp1 LT)
    //
    // Held → kicker UP. Released → kicker DOWN. Suppressed while a single-press
    // or triple-kick sequence is running so it can't fight the active sequence.
    // ═══════════════════════════════════════════════════════════════════════════

    private void handleKickerManual() {
        if (shooter.isKicking() || shooter.isTripleKicking()) return;

        if (gamepad1.left_trigger > 0.5) {
            shooter.kickerUp();
        } else {
            shooter.kickerDown();
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
    // Active-low: getState() == false means beam BROKEN.
    //
    // The sensor sits at the 3rd-ball slot, so when the robot is full the beam
    // stays continuously broken. We debounce the raw signal: clears shorter than
    // LASER_DEBOUNCE_OFF_SEC are treated as flicker. When the beam has been
    // stably broken for LASER_DETECT_RUMBLE_SEC, fire gamepad1.rumble(RUMBLE_MS)
    // once and light the RGB green. Re-arms when the beam is genuinely clear.
    // ═══════════════════════════════════════════════════════════════════════════

    private void handleLaserAndRgb() {
        boolean beamBroken = !laserInput.getState();

        if (beamBroken) {
            if (!debouncedBeamBroken) {
                // Transition: clear → broken. If the prior "clear" was shorter
                // than the debounce window, treat it as a flicker and KEEP the
                // existing brokenTimer running. Otherwise, start fresh.
                if (beamClearTimer.seconds() >= LASER_DEBOUNCE_OFF_SEC) {
                    beamBrokenTimer.reset();
                    rumbleFired = false;
                }
                debouncedBeamBroken = true;
            }

            if (!rumbleFired && beamBrokenTimer.seconds() >= LASER_DETECT_RUMBLE_SEC) {
                gamepad1.rumble(RUMBLE_MS);
                rumbleFired = true;
            }
        } else {
            if (debouncedBeamBroken) {
                // First tick of clear — start measuring the off-window.
                beamClearTimer.reset();
                debouncedBeamBroken = false;
            }
        }

        rgbLight.setPosition(debouncedBeamBroken ? RGB_GREEN : RGB_RED);
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