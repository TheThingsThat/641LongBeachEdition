package org.firstinspires.ftc.teamcode.configs;

import com.pedropathing.geometry.Pose;
import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class RobotConstants {

    // ─── Robot Poses ─────────────────────────────────────────────────────────
    public static Pose blueGoal = new Pose(6.5, 134, 0);
    public static Pose redGoal  = blueGoal.mirror();

    public static Pose blueReset = new Pose(132, 9.5, Math.toRadians(90)); // robot faces red goal
    public static Pose redReset  = blueReset.mirror();

    public static Pose blueAutoEnd = new Pose(63.9, 105.77, Math.toRadians(180));
    public static Pose redAutoEnd  = blueAutoEnd.mirror();

    public static Pose startPose = new Pose(21.0, 122.0, Math.toRadians(143.5));
    public static Pose scorePose = new Pose(52,    79.2,   Math.toRadians(180));
    public static Pose parkPose  = new Pose(63.9,  105.77, Math.toRadians(180));

    // Control points
    public static Pose control1        = new Pose(43.29, 104.73, Math.toRadians(180));
    public static Pose control2        = new Pose(54.73, 54.9);
    public static Pose control3        = new Pose(59.221, 79.143);
    public static Pose controlThirdRow = new Pose(55.95, 31.16);

    // Row pickup endpoints
    public static Pose pickup1Pose2 = new Pose(24.9, 34,   Math.toRadians(180));
    public static Pose pickup2Pose2 = new Pose(24.9, 58,   Math.toRadians(180));
    public static Pose pickup3Pose2 = new Pose(25.1, 83.3, Math.toRadians(180));

    // Gate poses
    public static Pose gateControl       = new Pose(44,   65.23);
    public static Pose gatePoseRotated   = new Pose(10.8, 58,   Math.toRadians(158));
    public static Pose gateRotateControl = new Pose(15.8, 61.7);
    public static Pose gatePose          = new Pose(17.4, 65.9, Math.toRadians(180));

    // ─── Hood ────────────────────────────────────────────────────────────────
    public static double hoodServoMax = 0.79;
    public static double hoodServoMin = 0.00;

    // ─── Flywheel / Shooter ──────────────────────────────────────────────────
    public static double FLYWHEEL_KP = 0.001;
    public static double FLYWHEEL_KI = 0.000001;
    public static double FLYWHEEL_KD = 0.0;
    public static double FLYWHEEL_KF = 0.0005;
    public static double SHOOTER_I_MAX = 0.4;

    // ─── Turret ──────────────────────────────────────────────────────────────
    public static double ppModeAngleTolerance = 0.5; // degrees
    public static double TICKS_PER_DEGREE = 773.9 / 360.0;

    public static double TURRET_KP = 0.045;
    public static double TURRET_KI = 0.01;
    public static double TURRET_KD = 0.0015;

    public static double TURRET_I_MAX     = 0.30;
    public static double TURRET_MAX_POWER = 1.0;
    public static double TURRET_MIN_POWER = 0.0;

    // ─── Auto-tracking offsets ───────────────────────────────────────────────
    public static double flywheelOffset = 0.0;
    public static double hoodOffset     = 0.0;

    // ─── Locked / Redundancy mode ────────────────────────────────────────────
    public static double lockedTurretDeg   = 0.0;
    public static double lockedHoodPos     = 0.37;
    public static double lockedFlywheelTPS = 1450.0;

    // ─── Fixed shooter positions (TURRET_ONLY and FULL_MANUAL modes) ─────────
    public static double fixedFlywheelTPS = 1550;
    public static double fixedHoodPos     = 0.25;

    // ─── Full-Adjustable mode step sizes ─────────────────────────────────────
    public static double adjustableHoodStep     = 0.01; // servo position per button press
    public static double adjustableFlywheelStep = 20.0; // TPS per button press

    // ─── Kicker ──────────────────────────────────────────────────────────────
    public static double kickerDownPos        = 0.35;
    public static double kickerUpPos          = 0.51;
    public static double kickerSinglePressSec = 1.0; // RB single-press up→down dwell
}