package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "Turret Auto Aim Test", group = "Test")
public class TurretAutoAimTest extends OpMode {
    private static final int PIPELINE_INDEX = 0;

    private static final double MIN_POSITION = 0.0;
    private static final double MAX_POSITION = 1.0;
    private static final double CENTER_POSITION = 0.5;

    private static final double STICK_DEADBAND = 0.08;
    private static final double MANUAL_SPEED = 0.10;

    // Servo-position change per second per degree of error.
    private static final double AIM_KP = 0.004;
    private static final double MAX_AIM_SPEED = 0.06;
    private static final double AIM_DEADBAND_DEG = 1.0;

    // Increasing servo position turns your turret LEFT.
    // Positive Limelight horizontal error means target is RIGHT.
    private static final double AIM_DIRECTION = -1.0;

    private static final long MAX_RESULT_AGE_MS = 150;
    private static final double LOCK_TIMEOUT_SECONDS = 0.75;

    private Servo rightServo;
    private Servo leftServo;
    private Limelight3A vision;

    private final ElapsedTime loopTimer = new ElapsedTime();
    private final ElapsedTime lostTimer = new ElapsedTime();

    private double position = CENTER_POSITION;
    private boolean autoAim;
    private boolean previousX;
    private int lockedId = -1;

    @Override
    public void init() {
        rightServo = hardwareMap.get(Servo.class, "rRotation");
        leftServo = hardwareMap.get(Servo.class, "lRotation");

        rightServo.setDirection(Servo.Direction.FORWARD);
        leftServo.setDirection(Servo.Direction.FORWARD);

        position = CENTER_POSITION;
        applyPosition();

        vision = hardwareMap.get(Limelight3A.class, "vision");
        vision.setPollRateHz(50);
        vision.pipelineSwitch(PIPELINE_INDEX);
        vision.start();

        autoAim = false;
        previousX = false;
        lockedId = -1;

        telemetry.addLine("Turret centered; automatic tracking OFF");
        telemetry.addLine("Controller 2 X: toggle tracking");
        telemetry.addLine("Left stick: manual override");
        telemetry.update();
    }

    @Override
    public void start() {
        loopTimer.reset();
        lostTimer.reset();
    }

    @Override
    public void loop() {
        double dt = Math.min(loopTimer.seconds(), 0.05);
        loopTimer.reset();

        if (gamepad2.x && !previousX) {
            autoAim = !autoAim;
            lockedId = -1;
            lostTimer.reset();
        }
        previousX = gamepad2.x;

        LLResult result = vision.getLatestResult();

        long ageMs = result == null ? -1 : result.getStaleness();

        boolean fresh = result != null
                && result.isValid()
                && ageMs >= 0
                && ageMs <= MAX_RESULT_AGE_MS
                && result.getPipelineIndex() == PIPELINE_INDEX;

        String state = "MANUAL";
        double errorDeg = Double.NaN;
        double commandSpeed = 0.0;
        double stick = gamepad2.left_stick_x;

        if (Math.abs(stick) > STICK_DEADBAND) {
            autoAim = false;
            lockedId = -1;

            double input = Math.signum(stick)
                    * (Math.abs(stick) - STICK_DEADBAND)
                    / (1.0 - STICK_DEADBAND);

            // Stick right decreases position, turning turret right.
            commandSpeed = -input * MANUAL_SPEED;
            state = "MANUAL OVERRIDE";
        } else if (autoAim) {
            if (lockedId != -1
                    && lostTimer.seconds() > LOCK_TIMEOUT_SECONDS) {
                lockedId = -1;
            }

            LLResultTypes.FiducialResult target = null;

            if (fresh && result.getFiducialResults() != null) {
                for (LLResultTypes.FiducialResult tag
                        : result.getFiducialResults()) {
                    if (!isEligible(tag)) {
                        continue;
                    }

                    if (lockedId != -1) {
                        if (tag.getFiducialId() == lockedId) {
                            target = tag;
                            break;
                        }
                    } else if (target == null
                            || Math.abs(tag.getTargetXDegrees())
                            < Math.abs(target.getTargetXDegrees())) {
                        target = tag;
                    }
                }
            }

            if (target == null) {
                state = "NO TARGET - HOLDING";
            } else {
                lockedId = target.getFiducialId();
                lostTimer.reset();
                errorDeg = target.getTargetXDegrees();

                if (Math.abs(errorDeg) <= AIM_DEADBAND_DEG) {
                    state = "CENTERED";
                } else {
                    // Subtract deadband for a gentle approach to center.
                    double effectiveError = Math.signum(errorDeg)
                            * (Math.abs(errorDeg) - AIM_DEADBAND_DEG);

                    commandSpeed = AIM_DIRECTION * Range.clip(
                            AIM_KP * effectiveError,
                            -MAX_AIM_SPEED,
                            MAX_AIM_SPEED
                    );
                    state = "TRACKING";
                }
            }
        }

        double requested = position + commandSpeed * dt;
        position = Range.clip(requested, MIN_POSITION, MAX_POSITION);

        if (requested < MIN_POSITION || requested > MAX_POSITION) {
            state = "TRAVEL LIMIT";
        }

        applyPosition();

        telemetry.addData("Auto aim", autoAim ? "ON" : "OFF");
        telemetry.addData("State", state);
        telemetry.addData("Locked tag", lockedId);
        telemetry.addData("Horizontal error deg", "%.2f", errorDeg);
        telemetry.addData("Result age ms", ageMs);
        telemetry.addData("Requested pipeline", PIPELINE_INDEX);
        telemetry.addData(
                "Reported pipeline",
                result == null ? -1 : result.getPipelineIndex()
        );
        telemetry.addData("Servo command", "%.3f", position);
        telemetry.addData("Command speed", "%.3f", commandSpeed);
        telemetry.update();
    }

    private boolean isEligible(LLResultTypes.FiducialResult tag) {
        int id = tag.getFiducialId();
        String family = tag.getFamily();

        return id >= 30 && id <= 37
                && family != null
                && family.contains("36h11")
                && Double.isFinite(tag.getTargetXDegrees());
    }

    private void applyPosition() {
        rightServo.setPosition(position);
        leftServo.setPosition(position);
    }

    @Override
    public void stop() {
        if (vision != null) {
            vision.stop();
        }
    }
}