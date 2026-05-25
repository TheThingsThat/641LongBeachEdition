package org.firstinspires.ftc.teamcode.configs;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.MathFunctions;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Shooter {

    // ─── Hardware ────────────────────────────────────────────────────────────
    public final DcMotor shooterR, shooterL, turretMotor;
    public final Servo   hoodServo;

    private final Follower  follower;
    private final Telemetry telemetry;

    // ─── Turret PID state ────────────────────────────────────────────────────
    private double turretIntegral  = 0.0;
    private double turretLastError = 0.0;
    private long   turretLastNs    = 0;

    // ─── Flywheel PID state ──────────────────────────────────────────────────
    private double pidIntegral       = 0.0;
    private double pidLastError      = 0.0;
    private double lastFlywheelPower = 0.0;
    private int    lastEncoderPos    = 0;
    private long   lastFlywheelNs    = 0;

    // ─── Calculated targets ──────────────────────────────────────────────────
    public int    turretTargetTicks  = 0;
    public double targetFlywheelTPS  = 0.0;
    public double targetHoodServoPos = 0.5;

    public int turretPhysicalOffset = 0;

    // ─── Telemetry outputs ───────────────────────────────────────────────────
    public double lastTurretRelDeg = 0.0;
    public double lastFlywheelTPS  = 0.0;

    public Shooter(HardwareMap hardwareMap, Follower follower, Telemetry telemetry) {
        this.follower  = follower;
        this.telemetry = telemetry;

        shooterR    = hardwareMap.get(DcMotor.class, "shooterR");
        shooterL    = hardwareMap.get(DcMotor.class, "shooterL");
        turretMotor = hardwareMap.get(DcMotor.class, "turretMotor");
        hoodServo   = hardwareMap.get(Servo.class,   "hoodServo");

        shooterR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterR.setDirection(DcMotorSimple.Direction.FORWARD);
        shooterL.setDirection(DcMotorSimple.Direction.REVERSE);
        shooterR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        lastEncoderPos = shooterR.getCurrentPosition();
        long now       = System.nanoTime();
        lastFlywheelNs = now;
        turretLastNs   = now;
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // TARGET CALCULATION
    // ═══════════════════════════════════════════════════════════════════════════

    /** Calculates turret, flywheel, and hood targets from a goal pose. */
    public void calculateTargets(Pose gatePose) {
        double dist = getRobotDistanceToGoal(gatePose);
        calculateTurretTarget(gatePose.getX(), gatePose.getY());
        targetFlywheelTPS  = flywheelSpeed(dist);
        targetHoodServoPos = hoodAngle(dist);
    }

    /** Sets turretTargetTicks from a field point. */
    public void calculateTurretTarget(double tx, double ty) {
        Pose   pose           = follower.getPose();
        double targetFieldRad = Math.atan2(ty - pose.getY(), tx - pose.getX());
        double turretRelRad   = wrapRad(targetFieldRad - pose.getHeading());

        lastTurretRelDeg  = Math.toDegrees(turretRelRad);
        turretTargetTicks = (int) Math.round(-lastTurretRelDeg * RobotConstants.TICKS_PER_DEGREE);
    }

    /** Flywheel speed (TPS) from distance (inches). */
    public static double flywheelSpeed(double goalDist) {
        double raw = 0.00514 * goalDist * goalDist
                + 4 * goalDist
                + 736;
        return MathFunctions.clamp(raw + RobotConstants.flywheelOffset, 0, 1250);
    }

    /** Hood servo position from distance (inches). */
    public static double hoodAngle(double goalDist) {
        double raw = 0.0000203 * goalDist * goalDist
                - 0.00925 * goalDist
                + 0.867;
        return MathFunctions.clamp(raw + RobotConstants.hoodOffset, RobotConstants.hoodServoMin, RobotConstants.hoodServoMax);
    }

    /** Straight-line distance (inches) from the robot to a field pose. */
    public double getRobotDistanceToGoal(Pose gatePose) {
        Pose   pose = follower.getPose();
        double dx   = gatePose.getX() - pose.getX();
        double dy   = gatePose.getY() - pose.getY();
        return Math.sqrt(dx * dx + dy * dy);
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // HARDWARE DRIVE
    // ═══════════════════════════════════════════════════════════════════════════

    /** Runs the turret PID toward turretTargetTicks. */
    public void driveTurret() {
        int currentTicks = turretMotor.getCurrentPosition();
        int errorTicks   = turretTargetTicks - currentTicks;

        long   nowNs = System.nanoTime();
        double dtSec = clamp((nowNs - turretLastNs) / 1e9, 0.0005, 0.05);
        turretLastNs = nowNs; // always update to prevent D-term spike on re-engagement

        double errorDeg = Math.abs(errorTicks) / RobotConstants.TICKS_PER_DEGREE;
        boolean aligned = errorDeg <= RobotConstants.ppModeAngleTolerance;

        double output = 0.0;
        if (aligned) {
            turretMotor.setPower(0.0);
            turretIntegral = 0.0;
        } else {
            double pTerm = RobotConstants.TURRET_KP * errorTicks;
            turretIntegral += errorTicks * dtSec;
            double iTerm = clamp(RobotConstants.TURRET_KI * turretIntegral,
                    -RobotConstants.TURRET_I_MAX, RobotConstants.TURRET_I_MAX);
            double dTerm = RobotConstants.TURRET_KD * (errorTicks - turretLastError) / dtSec;

            output = clamp(pTerm + iTerm + dTerm,
                    -RobotConstants.TURRET_MAX_POWER, RobotConstants.TURRET_MAX_POWER);
            if (Math.abs(output) < RobotConstants.TURRET_MIN_POWER) output = 0.0;
            turretMotor.setPower(output);
        }
        turretLastError = errorTicks;

        telemetry.addData("TurretRelDeg",  "%.1f", lastTurretRelDeg);
        telemetry.addData("TurretTarget",  "%d",   turretTargetTicks);
        telemetry.addData("TurretCurrent", "%d",   currentTicks);
        telemetry.addData("TurretOut",     "%.3f", output);
    }

    /** Manual turret override. Resets PID state so auto-tracking resumes cleanly. */
    public void driveManualTurret(double power) {
        turretMotor.setPower(power);
        turretIntegral  = 0.0;
        turretLastError = 0.0;
    }

    /**
     * Runs the flywheel velocity PID toward targetFlywheelTPS.
     * On a very fast loop (dt ≤ 1 ms) the last known power is re-applied so the
     * flywheel is never starved.
     */
    public double driveFlywheels(boolean shooterOn) {
        if (!shooterOn) {
            shooterR.setPower(0.0);
            shooterL.setPower(0.0);
            pidIntegral       = 0.0;
            pidLastError      = 0.0;
            lastFlywheelPower = 0.0;
            return 0.0;
        }

        long   nowNs = System.nanoTime();
        double dtSec = (nowNs - lastFlywheelNs) / 1e9;

        if (dtSec <= 0.001) {
            shooterR.setPower(lastFlywheelPower);
            shooterL.setPower(lastFlywheelPower);
            return lastFlywheelPower;
        }

        int    currentPos  = shooterR.getCurrentPosition();
        double velocityTPS = (currentPos - lastEncoderPos) / dtSec;
        lastFlywheelTPS    = velocityTPS;

        double error = targetFlywheelTPS - velocityTPS;
        double fTerm = RobotConstants.FLYWHEEL_KF * targetFlywheelTPS; // velocity feedforward
        double pTerm = RobotConstants.FLYWHEEL_KP * error;

        pidIntegral += error * dtSec;
        double iTerm = clamp(RobotConstants.FLYWHEEL_KI * pidIntegral,
                -RobotConstants.SHOOTER_I_MAX, RobotConstants.SHOOTER_I_MAX);

        double dTerm = RobotConstants.FLYWHEEL_KD * (error - pidLastError) / dtSec;
        double power = clamp(fTerm + pTerm + iTerm + dTerm, 0.0, 1.0);

        shooterR.setPower(power);
        shooterL.setPower(power);

        lastFlywheelPower = power;
        pidLastError      = error;
        lastEncoderPos    = currentPos;
        lastFlywheelNs    = nowNs;

        telemetry.addData("FlywheelTPS",    "%.1f", velocityTPS);
        telemetry.addData("FlywheelTarget", "%.1f", targetFlywheelTPS);
        telemetry.addData("FlywheelErr",    "%.1f", error);
        telemetry.addData("FlywheelPower",  "%.3f", power);

        return power;
    }

    /** Writes targetHoodServoPos to the hood servo. */
    public void driveHood() {
        double pos = clamp(targetHoodServoPos, RobotConstants.hoodServoMin, RobotConstants.hoodServoMax);
        hoodServo.setPosition(pos);
        telemetry.addData("HoodPos", "%.3f", pos);
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // LOCKED / REDUNDANCY MODE
    // ═══════════════════════════════════════════════════════════════════════════

    /** Locked mode: all three axes driven to fixed constants. */
    public void setLockedMode(boolean shooterOn) {
        turretTargetTicks = (int) Math.round(-RobotConstants.lockedTurretDeg * RobotConstants.TICKS_PER_DEGREE);
        turretIntegral    = 0.0;
        turretLastError   = 0.0;
        driveTurret();

        targetHoodServoPos = RobotConstants.lockedHoodPos;
        driveHood();

        targetFlywheelTPS = RobotConstants.lockedFlywheelTPS;
        driveFlywheels(shooterOn);
    }

    public void resetTurretEncoder() {
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turretTargetTicks = 0;
        turretIntegral    = 0.0;
        turretLastError   = 0.0;
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // HELPERS
    // ═══════════════════════════════════════════════════════════════════════════

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }

    private static double wrapRad(double rad) {
        while (rad >  Math.PI) rad -= 2.0 * Math.PI;
        while (rad < -Math.PI) rad += 2.0 * Math.PI;
        return rad;
    }
}
