package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.teamcode.subSystems.Rotation;
import org.firstinspires.ftc.teamcode.subSystems.Vision;
import org.firstinspires.ftc.teamcode.subSystems.Mecnum;

/** AprilTag camera tracking with manual flywheel, hood, and compression controls. */
@TeleOp(name = "TeleOp Red", group = "Test")
public class TeleOPRed extends OpMode {
    private static final String FLYWHEEL_NAME = "outake";
    private static final String HOOD_NAME = "hoodServo";
    private static final String VISION_SERVO_NAME = "visionServo";
    private static final String COMPRESSION_NAME = "compression";
    private static final String COMPRESSION_2_NAME = "compression2";

    private static final double TRIGGER_THRESHOLD = 0.1;
    private static final double HOOD_STEP_INTERVAL_SECONDS = 0.04;
    private static final double HOOD_MIN = 0.0;
    private static final double HOOD_MAX = 0.55;
    private static final double HOOD_STEP = 0.005;
    private static final double COMPRESSION_RETRACTED = 0.25;
    private static final double COMPRESSION_EXTENDED = 0.40;
    private static final double VISION_SERVO_TRAVEL = 0.15;
    private static final double VISION_TY_DEADBAND_DEG = 1.5;
    private static final double VISION_SERVO_GAIN = 0.0005;
    private static final double VISION_SERVO_MAX_STEP = 0.005;
    private static final double VISION_SERVO_SIGN = 1.0;

    private DcMotor flywheel;
    private Mecnum drive;
    private Servo hood;
    private Servo visionServo;
    private Servo compression;
    private Servo compression2;
    private Vision vision;
    private Rotation rotation;
    private boolean flywheelOn;
    private boolean autoAimEnabled;
    private boolean previousA;
    private boolean previousX;
    private double hoodPosition;
    private double visionServoPosition;
    private double visionServoMin;
    private double visionServoMax;
    private double compressionPosition;
    private final ElapsedTime hoodTimer = new ElapsedTime();

    @Override
    public void init() {
        flywheel = hardwareMap.get(DcMotor.class, FLYWHEEL_NAME);
        hood = hardwareMap.get(Servo.class, HOOD_NAME);
        visionServo = hardwareMap.get(Servo.class, VISION_SERVO_NAME);
        compression = hardwareMap.get(Servo.class, COMPRESSION_NAME);
        vision = new Vision(hardwareMap);
        rotation = new Rotation();
        rotation.init(hardwareMap);
        drive = new Mecnum(); drive.init(hardwareMap);

        try {
            compression2 = hardwareMap.get(Servo.class, COMPRESSION_2_NAME);
        } catch (IllegalArgumentException missingSecondServo) {
            compression2 = null;
        }

        flywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flywheel.setPower(0.0);
        flywheelOn = false;

        // Reading a servo position does not move it during init.
        hoodPosition = hood.getPosition();
        visionServoPosition = visionServo.getPosition();
        visionServoMin = Math.max(
                0.0, visionServoPosition - VISION_SERVO_TRAVEL);
        visionServoMax = Math.min(
                1.0, visionServoPosition + VISION_SERVO_TRAVEL);
        compressionPosition = compression.getPosition();
        hoodTimer.reset();
        telemetry.setMsTransmissionInterval(50);
        autoAimEnabled = false;
        previousA = false;
        previousX = false;

        telemetry.addLine(
                "Auto aim starts at Play | A: toggle | B: stop | X: clear tag lock");
        telemetry.addData(
                "Compression 2",
                compression2 == null ? "not configured" : "ready");
        telemetry.update();
    }

    @Override
    public void start() {
        rotation.clearTarget();
        vision.start();
        autoAimEnabled = true;
    }

    @Override
    public void loop() {
        if (gamepad1.b) {
            autoAimEnabled = false;
            rotation.stop();
        } else if (gamepad1.a && !previousA) {
            autoAimEnabled = !autoAimEnabled;
            if (autoAimEnabled) {
                rotation.clearTarget();
            } else {
                rotation.stop();
            }
        }
        previousA = gamepad1.a;

        if (gamepad1.x && !previousX) {
            rotation.clearTarget();
        }
        previousX = gamepad1.x;

        Vision.Frame frame = vision.update();
        Vision.TagObservation target =
                autoAimEnabled ? rotation.update(frame) : null;
        if (!autoAimEnabled) {
            rotation.stop();
        }

        // Left trigger stops; right trigger starts. State stays latched.
        if (gamepad1.left_trigger > TRIGGER_THRESHOLD) {
            flywheelOn = false;
        } else if (gamepad1.right_trigger > TRIGGER_THRESHOLD) {
            flywheelOn = true;
        }
        flywheel.setPower(flywheelOn ? 1.0 : 0.0);

        if (hoodTimer.seconds() >= HOOD_STEP_INTERVAL_SECONDS) {
            if (autoAimEnabled && target != null
                    && Double.isFinite(target.tyDeg)) {
                if (Math.abs(target.tyDeg) > VISION_TY_DEADBAND_DEG) {
                    double step = Range.clip(
                            VISION_SERVO_SIGN * VISION_SERVO_GAIN
                                    * target.tyDeg,
                            -VISION_SERVO_MAX_STEP,
                            VISION_SERVO_MAX_STEP);
                    visionServoPosition = Range.clip(
                            visionServoPosition + step,
                            visionServoMin,
                            visionServoMax);
                    visionServo.setPosition(visionServoPosition);
                }
            }

            if (gamepad1.dpad_up && !gamepad1.dpad_down) {
                setHood(hoodPosition + HOOD_STEP);
            } else if (gamepad1.dpad_down && !gamepad1.dpad_up) {
                setHood(hoodPosition - HOOD_STEP);
            }
            hoodTimer.reset();
        }

        if (gamepad1.left_bumper && !gamepad1.right_bumper) {
            setCompression(COMPRESSION_RETRACTED);
        } else if (gamepad1.right_bumper && !gamepad1.left_bumper) {
            setCompression(COMPRESSION_EXTENDED);
        }

        telemetry.addLine("APRILTAG CAMERA TRACKING");
        telemetry.addData(
                "Tracking", target != null ? "ACTIVE" : "INACTIVE");
        telemetry.addData(
                "Auto enabled", autoAimEnabled ? "YES" : "NO");
        telemetry.addData(
                "Reason",
                !autoAimEnabled ? "STOPPED (press A)"
                        : !frame.isUsable() ? frame.status.toString()
                          : target != null ? "TRACKING"
                            : rotation.getLockedTagId() >= 0
                              ? "LOCKED TAG NOT VISIBLE"
                              : "SEARCHING FOR 23/21/22");
        telemetry.addData(
                "Limelight",
                vision.isConnected() ? "CONNECTED" : "DISCONNECTED");
        telemetry.addData(
                "Pipeline", "%d (configured %d)",
                frame.reportedPipelineIndex,
                Vision.Config.APRILTAG_PIPELINE_INDEX);
        telemetry.addData(
                "Frame", "%s  age=%dms",
                frame.status, frame.stalenessMs);
        telemetry.addData("Reported tags", frame.tags.size());
        telemetry.addData(
                "Locked tag",
                rotation.getLockedTagId() < 0
                        ? "none" : rotation.getLockedTagId());

        if (target != null) {
            telemetry.addData(
                    "tx / ty (deg)", "%.1f / %.1f",
                    target.txDeg, target.tyDeg);
            telemetry.addData(
                    "X filtered / state", "%.1f / %s",
                    rotation.getFilteredTxDeg(),
                    rotation.getHorizontalState());
        } else if (!frame.tags.isEmpty()) {
            Vision.TagObservation first = frame.tags.get(0);
            telemetry.addData(
                    "First tag", "%d (%s) tx=%.1f ty=%.1f",
                    first.id, first.family,
                    first.txDeg, first.tyDeg);
        }

        if (!frame.rejected.isEmpty()) {
            telemetry.addData(
                    "Rejected tag", "%d: %s",
                    frame.rejected.get(0).id,
                    frame.rejected.get(0).reason);
        }

        telemetry.addData(
                "Rotation power / pivot", "%.3f / %.3f",
                rotation.getCommandedPower(),
                visionServoPosition);
        telemetry.update();
    }

    private void setHood(double requestedPosition) {
        hoodPosition = Range.clip(
                requestedPosition, HOOD_MIN, HOOD_MAX);
        hood.setPosition(hoodPosition);
    }

    private void setCompression(double requestedPosition) {
        compressionPosition = Range.clip(
                requestedPosition, 0.0, 1.0);
        compression.setPosition(compressionPosition);
        if (compression2 != null) {
            compression2.setDirection(Servo.Direction.REVERSE);
            compression2.setPosition(compressionPosition);
        }
    }

    @Override
    public void stop() {
        if (flywheel != null) {
            flywheel.setPower(0.0);
        }
        if (rotation != null) {
            rotation.stop();
        }
        if (vision != null) {
            vision.stop();
        }
        flywheelOn = false;

        drive.stop();
    }
}