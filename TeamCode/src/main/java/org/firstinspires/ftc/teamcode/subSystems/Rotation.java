package org.firstinspires.ftc.teamcode.subSystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * Turns an Axon MAX continuous-rotation servo toward one AprilTag.
 * Without a turret angle sensor, this cannot enforce travel limits.
 */
public class Rotation implements Subsystems {
    public static final String HARDWARE_NAME = "rotation";

    public static double STEER_SIGN = 1.0;
    public static double FILTER_ALPHA = 0.70;
    public static double START_MOVING_DEG = 7.0;
    public static double STOP_MOVING_DEG = 4.5;
    public static double FAR_ERROR_DEG = 18.0;
    public static double NEAR_POWER = 0.08;
    public static double FAR_POWER = 0.12;
    public static double NEAR_PULSE_SECONDS = 0.025;
    public static double FAR_PULSE_SECONDS = 0.060;
    public static double SETTLE_SECONDS = 0.30;

    private CRServo servo;
    private int lockedTagId = -1;
    private String lockedTagFamily = "";
    private double commandedPower;
    private double filteredTxDeg = Double.NaN;
    private boolean moving;
    private boolean pulseActive;
    private boolean settling;
    private double pulseDuration;
    private double pulseDirection;
    private final ElapsedTime pulseClock = new ElapsedTime();

    @Override
    public void init(HardwareMap hardwareMap) {
        servo = hardwareMap.get(CRServo.class, HARDWARE_NAME);
        stop();
    }

    public Vision.TagObservation update(Vision.Frame frame) {
        if (frame == null || !frame.isUsable()) {
            stop();
            return null;
        }

        if (lockedTagId < 0) {
            for (Vision.TagObservation tag : frame.tags) {
                if (isAllowedTag(tag.id)) {
                    lockedTagId = tag.id;
                    lockedTagFamily = tag.family;
                    break;
                }
            }
        }

        Vision.TagObservation target = null;
        for (Vision.TagObservation tag : frame.tags) {
            if (tag.id == lockedTagId
                    && tag.family.equals(lockedTagFamily)) {
                target = tag;
                break;
            }
        }

        if (target == null || !Double.isFinite(target.txDeg)) {
            stop();
            return null;
        }

        filteredTxDeg = Double.isNaN(filteredTxDeg)
                ? target.txDeg
                : FILTER_ALPHA * target.txDeg
                  + (1.0 - FILTER_ALPHA) * filteredTxDeg;
        double magnitude = Math.abs(filteredTxDeg);

        // The filtered reading can lag behind the turret. Stop immediately
        // if the raw camera reading reaches or crosses image center.
        boolean rawCentered =
                Math.abs(target.txDeg) <= STOP_MOVING_DEG;
        boolean rawCrossed = pulseActive
                && Math.signum(target.txDeg) != pulseDirection;

        if (rawCentered || rawCrossed
                || magnitude <= STOP_MOVING_DEG) {
            if (moving || pulseActive) {
                settling = true;
                pulseClock.reset();
            }

            moving = false;
            pulseActive = false;

            if (settling
                    && pulseClock.seconds() >= SETTLE_SECONDS) {
                settling = false;
            }

            commandedPower = 0.0;
            servo.setPower(0.0);
            return target;
        }

        if (pulseActive) {
            if (pulseClock.seconds() < pulseDuration) {
                servo.setPower(commandedPower);
                return target;
            }

            pulseActive = false;
            settling = true;
            pulseClock.reset();
        }

        commandedPower = 0.0;

        if (settling
                && pulseClock.seconds() < SETTLE_SECONDS) {
            servo.setPower(0.0);
            return target;
        }

        settling = false;

        if (!moving && magnitude < START_MOVING_DEG) {
            servo.setPower(0.0);
            return target;
        }

        moving = true;

        double fraction = Range.clip(
                (magnitude - STOP_MOVING_DEG)
                        / (FAR_ERROR_DEG - STOP_MOVING_DEG),
                0.0,
                1.0);

        pulseDuration = NEAR_PULSE_SECONDS
                + fraction * (FAR_PULSE_SECONDS
                - NEAR_PULSE_SECONDS);
        double power = NEAR_POWER
                + fraction * (FAR_POWER - NEAR_POWER);

        pulseDirection = Math.signum(filteredTxDeg);
        commandedPower =
                Math.copySign(power, STEER_SIGN * filteredTxDeg);
        pulseActive = true;
        pulseClock.reset();
        servo.setPower(commandedPower);

        return target;
    }

    private static boolean isAllowedTag(int id) {
        return id == 23 || id == 21 || id == 22;
    }

    public void clearTarget() {
        lockedTagId = -1;
        lockedTagFamily = "";
        stop();
    }

    public int getLockedTagId() {
        return lockedTagId;
    }

    public String getLockedTagFamily() {
        return lockedTagFamily;
    }

    public double getCommandedPower() {
        return commandedPower;
    }

    public double getFilteredTxDeg() {
        return filteredTxDeg;
    }

    public String getHorizontalState() {
        if (pulseActive) return "CORRECTING";
        if (settling) return "WAITING FOR CAMERA";
        return moving ? "WAITING" : "CENTERED / IDLE";
    }

    @Override
    public void stop() {
        commandedPower = 0.0;
        filteredTxDeg = Double.NaN;
        moving = false;
        pulseActive = false;
        settling = false;

        if (servo != null) {
            servo.setPower(0.0);
        }
    }
}