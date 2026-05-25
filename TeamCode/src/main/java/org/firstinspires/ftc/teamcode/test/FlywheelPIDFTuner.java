package org.firstinspires.ftc.teamcode.test;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * Flywheel PIDF tuner.
 *
 * Public static KP / KI / KD / KF / TARGET_TPS / I_MAX are exposed via Panels
 * Configurables — edit them live in the dashboard while the OpMode runs.
 *
 * Gamepad1 controls:
 *   A             — toggle flywheel on/off
 *   D-pad up      — TARGET_TPS += 50
 *   D-pad down    — TARGET_TPS -= 50
 *   D-pad right   — TARGET_TPS += 250
 *   D-pad left    — TARGET_TPS -= 250
 *   B             — zero the I accumulator (panic reset)
 *
 * Telemetry exposes each PIDF component separately so you can see whether the
 * feedforward alone is close to target and how much the PID is contributing.
 *
 * Tuning workflow:
 *   1. Set KI = KD = 0. Sweep KF until the wheel free-spins close to TARGET_TPS.
 *   2. Add a small KP to clean up steady-state error.
 *   3. Add KI to handle load disturbances (firing a ball).
 *   4. KD only if there's overshoot you want to damp.
 */
@Configurable
@TeleOp(name = "Flywheel Tuner")
public class FlywheelPIDFTuner extends OpMode {

    // ─── Live-tunable gains (edit via Panels) ────────────────────────────────
    public static double KP         = 0.045;
    public static double KI         = 0.3;
    public static double KD         = 0.001;
    public static double KF         = 1.0 / 3000.0;
    public static double I_MAX      = 0.4;
    public static double TARGET_TPS = 1500.0;

    // ─── Hardware ────────────────────────────────────────────────────────────
    private DcMotor shooterR, shooterL;

    // ─── Panels telemetry ────────────────────────────────────────────────────
    private TelemetryManager panels;

    // ─── PID state ───────────────────────────────────────────────────────────
    private double pidIntegral       = 0.0;
    private double pidLastError      = 0.0;
    private double lastFlywheelPower = 0.0;
    private int    lastEncoderPos    = 0;
    private long   lastFlywheelNs    = 0;

    // ─── Telemetry / runtime state ───────────────────────────────────────────
    private boolean shooterOn = true;
    private boolean lastGp1A  = false;
    private boolean lastGp1B  = false;
    private boolean lastDUp    = false;
    private boolean lastDDown  = false;
    private boolean lastDLeft  = false;
    private boolean lastDRight = false;

    // ═══════════════════════════════════════════════════════════════════════════
    // INIT
    // ═══════════════════════════════════════════════════════════════════════════

    @Override
    public void init() {
        panels = PanelsTelemetry.INSTANCE.getTelemetry();

        shooterR = hardwareMap.get(DcMotor.class, "shooterR");
        shooterL = hardwareMap.get(DcMotor.class, "shooterL");

        shooterR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterR.setDirection(DcMotorSimple.Direction.FORWARD);
        shooterL.setDirection(DcMotorSimple.Direction.REVERSE);
        shooterR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        lastEncoderPos = shooterR.getCurrentPosition();
        lastFlywheelNs = System.nanoTime();

        telemetry.addLine("Flywheel PIDF Tuner ready.");
        telemetry.update();
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // LOOP
    // ═══════════════════════════════════════════════════════════════════════════

    @Override
    public void loop() {
        handleGamepad();

        double power = updateFlywheelPID();
        shooterR.setPower(power);
        shooterL.setPower(power);
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // GAMEPAD
    // ═══════════════════════════════════════════════════════════════════════════

    private void handleGamepad() {
        boolean gp1A = gamepad1.a;
        if (gp1A && !lastGp1A) shooterOn = !shooterOn;
        lastGp1A = gp1A;

        boolean gp1B = gamepad1.b;
        if (gp1B && !lastGp1B) {
            pidIntegral  = 0.0;
            pidLastError = 0.0;
        }
        lastGp1B = gp1B;

        boolean dUp    = gamepad1.dpad_up;
        boolean dDown  = gamepad1.dpad_down;
        boolean dLeft  = gamepad1.dpad_left;
        boolean dRight = gamepad1.dpad_right;

        if (dUp    && !lastDUp)    TARGET_TPS += 50.0;
        if (dDown  && !lastDDown)  TARGET_TPS -= 50.0;
        if (dRight && !lastDRight) TARGET_TPS += 250.0;
        if (dLeft  && !lastDLeft)  TARGET_TPS -= 250.0;

        if (TARGET_TPS < 0) TARGET_TPS = 0;

        lastDUp    = dUp;
        lastDDown  = dDown;
        lastDLeft  = dLeft;
        lastDRight = dRight;
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // PIDF
    // ═══════════════════════════════════════════════════════════════════════════

    private double updateFlywheelPID() {
        if (!shooterOn) {
            shooterR.setPower(0.0);
            shooterL.setPower(0.0);
            pidIntegral       = 0.0;
            pidLastError      = 0.0;
            lastFlywheelPower = 0.0;
            pushTelemetry(0, 0, 0, 0, 0, 0, 0);
            return 0.0;
        }

        long   nowNs = System.nanoTime();
        double dtSec = (nowNs - lastFlywheelNs) / 1e9;

        if (dtSec <= 0.001) {
            return lastFlywheelPower;
        }

        int    currentPos  = shooterR.getCurrentPosition();
        double velocityTPS = (currentPos - lastEncoderPos) / dtSec;

        double error = TARGET_TPS - velocityTPS;
        double fTerm = KF * TARGET_TPS;
        double pTerm = KP * error;

        pidIntegral += error * dtSec;
        double iTerm = clamp(KI * pidIntegral, -I_MAX, I_MAX);

        double dTerm = KD * (error - pidLastError) / dtSec;
        double power = clamp(fTerm + pTerm + iTerm + dTerm, 0.0, 1.0);

        lastFlywheelPower = power;
        pidLastError      = error;
        lastEncoderPos    = currentPos;
        lastFlywheelNs    = nowNs;

        pushTelemetry(velocityTPS, error, fTerm, pTerm, iTerm, dTerm, power);
        return power;
    }

    private void pushTelemetry(double velocityTPS, double error,
                               double fTerm, double pTerm, double iTerm, double dTerm,
                               double power) {
        telemetry.addData("ShooterOn",  shooterOn);
        telemetry.addData("Target TPS", "%.0f", TARGET_TPS);
        telemetry.addData("Actual TPS", "%.1f", velocityTPS);
        telemetry.addData("Error",      "%.1f", error);
        telemetry.addData("fTerm",      "%.4f", fTerm);
        telemetry.addData("pTerm",      "%.4f", pTerm);
        telemetry.addData("iTerm",      "%.4f", iTerm);
        telemetry.addData("dTerm",      "%.4f", dTerm);
        telemetry.addData("Power",      "%.4f", power);
        telemetry.addData("KP / KI / KD / KF",
                "%.4f / %.4f / %.4f / %.5f", KP, KI, KD, KF);

        panels.debug("Target TPS", TARGET_TPS);
        panels.debug("Actual TPS", velocityTPS);
        panels.debug("Error",      error);
        panels.debug("fTerm",      fTerm);
        panels.debug("pTerm",      pTerm);
        panels.debug("iTerm",      iTerm);
        panels.debug("dTerm",      dTerm);
        panels.debug("Power",      power);
        panels.update(telemetry);
    }

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }
}
