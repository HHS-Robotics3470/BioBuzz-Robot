package org.firstinspires.ftc.teamcode.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.subSystems.vision;

import java.util.ArrayList;
import java.util.List;

/**
 * STANDALONE TURRET TEST ----- NOTHING SHOULD BE COPIED INTO PROD
 * GAMEPAD 1 CONTROLS:
 *   D-Pad Up            - switch to MANUAL mode
 *   D-Pad Down          - switch to VISION MONITOR mode (camera-only, never moves the turret)
 *   Right Bumper        - request AUTO mode (only engages once limits are configured AND
 *                          validated AND an alliance is selected; otherwise ignored)
 *   D-Pad Left/Right    - (MANUAL mode only) rotate turret left/right at MANUAL_POWER
 *                          while held; released = stopped immediately
 *   B                   - select RED alliance
 *   X                   - select BLUE alliance
 *   Back                - EMERGENCY STOP: forces MANUAL mode and zero power immediately,
 *                          from any mode, for as long as it is held
 */
@TeleOp(name = "TEST: Turret Hardware", group = "Test")
public class TurretHardwareTestOpMode extends LinearOpMode {

    private enum TestMode { MANUAL, VISION_MONITOR, AUTO }

    private enum AutoState { REFUSED, TRACKING, RETURNING }

    /** Fuse every tag into one cluster. Will prevent tracking oposing alliance tags.
     * */
    private static final class ClusterFusion {
        final int[] tagIds;
        final double avgTxDeg;
        final double avgTyDeg;

        ClusterFusion(int[] tagIds, double avgTxDeg, double avgTyDeg) {
            this.tagIds = tagIds;
            this.avgTxDeg = avgTxDeg;
            this.avgTyDeg = avgTyDeg;
        }
    }

    private DcMotor aimMotor;
    private vision visionSubsystem;

    private int forwardTicks;
    private TestMode mode = TestMode.MANUAL;
    private vision.Alliance selectedAlliance = vision.Alliance.NONE;

    // Rising-edge tracking for mode/alliance buttons which prevents a held button from re-triggering.
    private boolean prevDpadUp, prevDpadDown, prevRightBumper, prevB, prevX;

    private String autoRefusedReason = "";

    @Override
    public void runOpMode() {
        aimMotor = hardwareMap.get(DcMotor.class, TurretHardwareTestConfig.HARDWARE_NAME_AIM_MOTOR);
        aimMotor.setDirection(TurretHardwareTestConfig.AIM_MOTOR_REVERSED
                ? DcMotorSimple.Direction.REVERSE
                : DcMotorSimple.Direction.FORWARD);
        aimMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        aimMotor.setZeroPowerBehavior(TurretHardwareTestConfig.BRAKE_AT_ZERO_POWER
                ? DcMotor.ZeroPowerBehavior.BRAKE
                : DcMotor.ZeroPowerBehavior.FLOAT);
        aimMotor.setPower(0.0);

        forwardTicks = aimMotor.getCurrentPosition();

        visionSubsystem = new vision(hardwareMap);
        visionSubsystem.start();

        telemetry.addLine("Turret hardware test ready.");
        telemetry.addData("Forward reference (ticks)", forwardTicks);
        telemetry.addData("Limits configured", limitsValidAndConfigured());
        telemetry.addLine("Confirm the turret is pointed FORWARD before pressing Play.");
        telemetry.update();

        waitForStart();

        try {
            while (opModeIsActive()) {
                runLoopIteration();
            }
        } finally {
            // Always leave the motor stopped and the camera cleanly shut down, whether
            // the OpMode ended normally, was stopped by the driver, or threw.
            aimMotor.setPower(0.0);
            visionSubsystem.stop();
        }
    }

    private void runLoopIteration() {
        boolean emergencyStop = gamepad1.back;
        boolean modeJustChanged = handleModeAndAllianceInput(emergencyStop);

        // Read the Limelight exactly once this loop. Every consumer below (telemetry
        // and AUTO steering) reads this SAME frame - it is never re-fetched, so vision
        // monitoring and AUTO steering can never disagree about what the camera saw.
        vision.Frame frame = visionSubsystem.update();

        // Encoder-derived state. This is a completely separate error...
        // from the camera's bearing error above and must never be substituted for it
        // Compare turret position and turret target position.
        int currentTicks = aimMotor.getCurrentPosition();
        int relativeTicks = currentTicks - forwardTicks;

        double commandedPower = 0.0;
        AutoState autoState = AutoState.REFUSED;
        ClusterFusion tracked = null;

        if (emergencyStop) {
            commandedPower = 0.0;
        } else if (modeJustChanged) {
            // No turret motion during a mode transition - cycle only.
            commandedPower = 0.0;
        } else {
            switch (mode) {
                case MANUAL: {
                    double manual = 0.0;
                    if (gamepad1.dpad_left) {
                        manual = -TurretHardwareTestConfig.MANUAL_POWER;
                    } else if (gamepad1.dpad_right) {
                        manual = TurretHardwareTestConfig.MANUAL_POWER;
                    }
                    commandedPower = enforceLimits(manual, relativeTicks);
                    break;
                }
                case VISION_MONITOR: {
                    // Explicitly zero: the Limelight must never move the turret here.
                    commandedPower = 0.0;
                    break;
                }
                case AUTO: {
                    if (!canEngageAuto()) {
                        commandedPower = 0.0;
                        autoState = AutoState.REFUSED;
                        break;
                    }

                    tracked = fuseBestCluster(frame, selectedAlliance);
                    if (frame.isUsable() && tracked != null) {
                        double bearingErrorDeg = tracked.avgTxDeg;
                        double steer;
                        if (Math.abs(bearingErrorDeg) < TurretHardwareTestConfig.AUTO_DEADBAND_DEG) {
                            steer = 0.0;
                        } else {
                            steer = clamp(TurretHardwareTestConfig.AUTO_STEER_KP * bearingErrorDeg,
                                    -TurretHardwareTestConfig.AUTO_MAX_POWER,
                                    TurretHardwareTestConfig.AUTO_MAX_POWER);
                        }
                        commandedPower = enforceLimits(steer, relativeTicks);
                        autoState = AutoState.TRACKING;
                    } else {
                        // Vision failure OR no usable alliance target: return slowly to inital position
                        // toward the startup forward position using ENCODER position
                        double ret;
                        if (Math.abs(relativeTicks) <= TurretHardwareTestConfig.AUTO_RETURN_DEADBAND_TICKS) {
                            ret = 0.0;
                        } else {
                            ret = relativeTicks > 0
                                    ? -TurretHardwareTestConfig.AUTO_RETURN_POWER
                                    : TurretHardwareTestConfig.AUTO_RETURN_POWER;
                        }
                        commandedPower = enforceLimits(ret, relativeTicks);
                        autoState = AutoState.RETURNING;
                    }
                    break;
                }
            }
        }

        commandedPower = clamp(commandedPower, -1.0, 1.0);
        aimMotor.setPower(commandedPower);

        publishTelemetry(frame, currentTicks, relativeTicks, commandedPower, autoState, tracked, emergencyStop);
    }

    private boolean handleModeAndAllianceInput(boolean emergencyStop) {
        boolean dpadUp = gamepad1.dpad_up;
        boolean dpadDown = gamepad1.dpad_down;
        boolean rightBumper = gamepad1.right_bumper;
        boolean bButton = gamepad1.b;
        boolean xButton = gamepad1.x;

        boolean upPressed = dpadUp && !prevDpadUp;
        boolean downPressed = dpadDown && !prevDpadDown;
        boolean rbPressed = rightBumper && !prevRightBumper;
        boolean bPressed = bButton && !prevB;
        boolean xPressed = xButton && !prevX;

        prevDpadUp = dpadUp;
        prevDpadDown = dpadDown;
        prevRightBumper = rightBumper;
        prevB = bButton;
        prevX = xButton;

        if (bPressed) {
            selectedAlliance = vision.Alliance.RED;
        } else if (xPressed) {
            selectedAlliance = vision.Alliance.BLUE;
        }

        TestMode before = mode;

        if (emergencyStop) {
            mode = TestMode.MANUAL;
        } else if (upPressed) {
            mode = TestMode.MANUAL;
        } else if (downPressed) {
            mode = TestMode.VISION_MONITOR;
        } else if (rbPressed) {
            if (canEngageAuto()) {
                mode = TestMode.AUTO;
                autoRefusedReason = "";
            } else {
                autoRefusedReason = buildAutoRefusedReason();
                // Stay in the current mode; AUTO refuses to engage.
            }
        }

        return mode != before;
    }

    private boolean canEngageAuto() {
        return limitsValidAndConfigured() && selectedAlliance != vision.Alliance.NONE;
    }

    private String buildAutoRefusedReason() {
        StringBuilder sb = new StringBuilder("AUTO refused: ");
        boolean needsComma = false;
        if (!limitsValidAndConfigured()) {
            sb.append("encoder limits not configured/validated");
            needsComma = true;
        }
        if (selectedAlliance == vision.Alliance.NONE) {
            if (needsComma) sb.append(", ");
            sb.append("no alliance selected (press B or X)");
        }
        return sb.toString();
    }

    private static boolean limitsValidAndConfigured() {
        int left = TurretHardwareTestConfig.LEFT_LIMIT_TICKS;
        int right = TurretHardwareTestConfig.RIGHT_LIMIT_TICKS;
        return TurretHardwareTestConfig.LIMITS_CONFIGURED
                && left != TurretHardwareTestConfig.LIMIT_NOT_CONFIGURED
                && right != TurretHardwareTestConfig.LIMIT_NOT_CONFIGURED
                && left < 0
                && right > 0
                && Math.abs(left) <= TurretHardwareTestConfig.LIMIT_SANITY_MAX_TICKS
                && Math.abs(right) <= TurretHardwareTestConfig.LIMIT_SANITY_MAX_TICKS;
    }

    /** Blocks motion that would push farther past a limit that has already been
     *  reached or exceeded. If limits are not yet configured, motion is NOT restricted
     *  here (that would make the calibration procedure in MANUAL mode impossible) -
     *  the caller/telemetry must make that unrestricted state obvious to the driver. */
    private double enforceLimits(double requestedPower, int relativeTicks) {
        if (!limitsValidAndConfigured()) {
            return requestedPower;
        }
        if (relativeTicks <= TurretHardwareTestConfig.LEFT_LIMIT_TICKS && requestedPower < 0) {
            return 0.0;
        }
        if (relativeTicks >= TurretHardwareTestConfig.RIGHT_LIMIT_TICKS && requestedPower > 0) {
            return 0.0;
        }
        return requestedPower;
    }

    /**
     * Fuses every currently-visible tag belonging to ONE physical CELL cluster of the
     * given alliance. Tags from different clusters, or from the other alliance, are
     * never combined into the same fused bearing.
     * Returns null if the frame is not usable or no tag of the selected alliance is visible.
     */
    private static ClusterFusion fuseBestCluster(vision.Frame frame, vision.Alliance alliance) {
        if (!frame.isUsable() || alliance == vision.Alliance.NONE) {
            return null;
        }

        ClusterFusion best = null;
        int bestCount = 0;

        for (int[] group : TurretHardwareTestConfig.CELL_TAG_GROUPS_PLACEHOLDER_VERIFY_BEFORE_USE) {
            if (vision.allianceForId(group[0]) != alliance) {
                continue; // this group belongs to the other alliance - skip entirely
            }
            List<Integer> seenIds = new ArrayList<>();
            List<Double> txs = new ArrayList<>();
            List<Double> tys = new ArrayList<>();
            for (vision.TagObservation t : frame.tags) {
                if (t.alliance != alliance) {
                    continue; // never mix alliances into one fused bearing
                }
                for (int id : group) {
                    if (t.id == id) {
                        seenIds.add(t.id);
                        txs.add(t.txDeg);
                        tys.add(t.tyDeg);
                        break;
                    }
                }
            }
            if (seenIds.isEmpty()) {
                continue;
            }
            double avgTx = average(txs);
            if (seenIds.size() > bestCount
                    || (best != null && seenIds.size() == bestCount && Math.abs(avgTx) < Math.abs(best.avgTxDeg))) {
                int[] idsArray = new int[seenIds.size()];
                for (int i = 0; i < idsArray.length; i++) idsArray[i] = seenIds.get(i);
                best = new ClusterFusion(idsArray, avgTx, average(tys));
                bestCount = seenIds.size();
            }
        }

        if (best != null) {
            return best;
        }

        // Fallback: an alliance tag was seen that isn't in the placeholder grouping
        // table above (e.g. the table is wrong/incomplete). Track it alone rather than
        // refusing to move at all, but never merge it with any other tag.
        for (vision.TagObservation t : frame.tags) {
            if (t.alliance == alliance) {
                return new ClusterFusion(new int[]{t.id}, t.txDeg, t.tyDeg);
            }
        }
        return null;
    }

    private static double average(List<Double> values) {
        double sum = 0.0;
        for (double v : values) sum += v;
        return sum / values.size();
    }

    private static double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    private void publishTelemetry(vision.Frame frame, int currentTicks, int relativeTicks,
                                  double commandedPower, AutoState autoState,
                                  ClusterFusion tracked, boolean emergencyStop) {
        telemetry.addLine("--- TURRET HARDWARE TEST ---");
        if (emergencyStop) {
            telemetry.addLine("*** EMERGENCY STOP HELD (Back) ***");
        }
        telemetry.addData("Mode", mode);
        telemetry.addData("Alliance", selectedAlliance == vision.Alliance.NONE
                ? "NOT SELECTED (press B=red, X=blue)" : selectedAlliance);
        telemetry.addData("Encoder (abs / rel to fwd)", "%d / %d", currentTicks, relativeTicks);
        telemetry.addData("Commanded power", "%.3f", commandedPower);
        if (limitsValidAndConfigured()) {
            telemetry.addData("Soft limits", "L=%d  R=%d  (configured+validated)",
                    TurretHardwareTestConfig.LEFT_LIMIT_TICKS, TurretHardwareTestConfig.RIGHT_LIMIT_TICKS);
        } else {
            telemetry.addLine("Soft limits: NOT CONFIGURED - manual motion is NOT limit-clamped. "
                    + "Rotate cautiously and release the button before any hard stop.");
        }

        switch (mode) {
            case MANUAL:
                telemetry.addLine("--- MANUAL: hold D-Pad Left/Right to rotate. Release = stop. ---");
                break;
            case VISION_MONITOR:
                telemetry.addLine("--- VISION MONITOR: camera only, turret will not move ---");
                telemetry.addData("Frame status", "%s  usable=%b  age=%dms",
                        frame.status, frame.isUsable(), frame.stalenessMs);
                if (selectedAlliance == vision.Alliance.NONE) {
                    telemetry.addData("Tags (all, unfiltered)", frame.tags.size());
                    telemetry.addLine("Select an alliance (B/X) to filter to our tags.");
                } else {
                    int shown = 0;
                    for (vision.TagObservation t : frame.tags) {
                        if (t.alliance != selectedAlliance) {
                            continue; // opposing-alliance tags are filtered out of this view
                        }
                        telemetry.addData("  tag " + t.id, "alliance=%s tx=%.1fdeg ty=%.1fdeg",
                                t.alliance, t.txDeg, t.tyDeg);
                        shown++;
                    }
                    if (shown == 0) {
                        telemetry.addLine("No " + selectedAlliance + " tags currently visible.");
                    }
                }
                visionSubsystem.addTelemetry(telemetry); // raw diagnostic dump
                break;
            case AUTO:
                telemetry.addLine("-- AUTO: TRACKING TEST ONLY - NOT calibrated shooter aim --");
                if (!canEngageAuto()) {
                    telemetry.addLine(autoRefusedReason.isEmpty() ? buildAutoRefusedReason() : autoRefusedReason);
                } else {
                    telemetry.addData("State", autoState);
                    if (tracked != null) {
                        StringBuilder ids = new StringBuilder();
                        for (int id : tracked.tagIds) {
                            if (ids.length() > 0) ids.append(',');
                            ids.append(id);
                        }
                        telemetry.addData("Tracked cluster tags", "[%s] (%d)", ids, tracked.tagIds.length);
                        telemetry.addData("Fused bearing (tx)", "%.2f deg", tracked.avgTxDeg);
                    } else {
                        telemetry.addLine("No usable " + selectedAlliance + " cluster - returning toward forward.");
                    }
                }
                break;
        }

        telemetry.update();
    }
}