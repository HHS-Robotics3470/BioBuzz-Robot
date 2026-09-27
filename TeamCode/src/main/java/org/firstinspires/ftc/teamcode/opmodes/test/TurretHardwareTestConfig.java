package org.firstinspires.ftc.teamcode.opmodes.test;

/**
 * Tuning values and hardware names for TurretHardware ONLY.
 *
 * STANDALONE first physical-test rig. NOT PROD
 * All values below are PROVISIONAL until confirmed on the real robot. Nothing here may
 * be copied into production turret code without re-deriving/re-measuring it there.
 */
public final class TurretHardwareTestConfig {

    private TurretHardwareTestConfig() {
    }
    public static final String HARDWARE_NAME_AIM_MOTOR = "aimMotor";
    // which owns its own hardware name constant.)
    // Motor direction / sign convention
    public static final boolean AIM_MOTOR_REVERSED = false;
    public static final boolean BRAKE_AT_ZERO_POWER = true;

    // MANUAL aim mode

    /** Default open-loop power while a manual rotate button is held. Tunable. */
    public static final double MANUAL_POWER = 0.15;

    // Encoder limits (if applicable)
    public static final int LIMIT_NOT_CONFIGURED = Integer.MIN_VALUE;
    public static final int LEFT_LIMIT_TICKS = LIMIT_NOT_CONFIGURED;
    public static final int RIGHT_LIMIT_TICKS = LIMIT_NOT_CONFIGURED;
    public static final boolean LIMITS_CONFIGURED = false;
    public static final int LIMIT_SANITY_MAX_TICKS = 5000;

    // AUTO
    public static final double AUTO_MAX_POWER = 0.20;
    public static final double AUTO_STEER_KP = 0.01;
    public static final double AUTO_DEADBAND_DEG = 1.5;
    public static final double AUTO_RETURN_POWER = 0.10;
    public static final int AUTO_RETURN_DEADBAND_TICKS = 15;

    public static final int RED_CELL_ID_MIN = 30;
    public static final int RED_CELL_ID_MAX = 37;
    public static final int BLUE_CELL_ID_MIN = 38;
    public static final int BLUE_CELL_ID_MAX = 45;

    /**
     * PLACEHOLDER tag-to-CELL clustering, grouped by simple ID-range splitting (four
     * consecutive IDs per CELL, per ROBOT_CONTEXT.md's "four IDs per CELL cluster").
     */
    public static final int[][] CELL_TAG_GROUPS_PLACEHOLDER_VERIFY_BEFORE_USE = {
            {30, 31, 32, 33},
            {34, 35, 36, 37},
            {38, 39, 40, 41},
            {42, 43, 44, 45},
    };
}