package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.subSystems.Vision;

import java.util.List;

/** Camera-only AprilTag diagnostics; this OpMode never moves the servos. */
@TeleOp(name = "Vision Test", group = "Test")
public class VisionTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        Limelight3A camera = hardwareMap.get(
                Limelight3A.class, Vision.Config.HARDWARE_NAME
        );
        camera.setPollRateHz(Vision.Config.POLL_RATE_HZ);
        camera.pipelineSwitch(Vision.Config.APRILTAG_PIPELINE_INDEX);
        telemetry.setMsTransmissionInterval(50);

        telemetry.addLine("VISION TEST - camera only. Press Play.");
        telemetry.update();
        waitForStart();
        if (isStopRequested()) return;

        camera.start();
        try {
            while (opModeIsActive()) {
                telemetry.addLine("APRILTAG VISION TEST");
                telemetry.addData("Limelight", camera.isConnected()
                        ? "CONNECTED" : "DISCONNECTED");

                LLResult result = camera.getLatestResult();
                if (result == null) {
                    telemetry.addData("Frame", "NO_RESULT");
                    telemetry.addData("Reported tags", 0);
                    telemetry.update();
                    idle();
                    continue;
                }

                int pipeline = result.getPipelineIndex();
                long ageMs = result.getStaleness();
                List<LLResultTypes.FiducialResult> tags =
                        result.getFiducialResults();
                int tagCount = tags == null ? 0 : tags.size();

                String status = !camera.isConnected() ? "DISCONNECTED"
                        : !result.isValid() ? "INVALID_RESULT"
                          : pipeline != Vision.Config.APRILTAG_PIPELINE_INDEX
                            ? "WRONG_PIPELINE"
                            : ageMs < 0
                        || ageMs > Vision.Config.MAX_STALENESS_MS
                              ? "STALE" : "OK";

                telemetry.addData("Frame", "%s  age=%dms", status, ageMs);
                telemetry.addData("Pipeline", "%d (configured %d)",
                        pipeline,
                        Vision.Config.APRILTAG_PIPELINE_INDEX);
                telemetry.addData("Reported tags", tagCount);
                telemetry.addData("Tracking ready",
                        "OK".equals(status) && tagCount > 0
                                ? "YES" : "NO");

                if (tags != null) {
                    for (LLResultTypes.FiducialResult tag : tags) {
                        if (tag == null) continue;

                        int id = tag.getFiducialId();
                        telemetry.addData("Tag " + id,
                                "family=%s tx=%.1f ty=%.1f",
                                tag.getFamily(),
                                tag.getTargetXDegrees(),
                                tag.getTargetYDegrees());

                        Pose3D tagPose = tag.getTargetPoseCameraSpace();
                        if (tagPose != null
                                && tagPose.getPosition() != null) {
                            Position p = tagPose.getPosition()
                                    .toUnit(DistanceUnit.INCH);

                            if (Double.isFinite(p.x)
                                    && Double.isFinite(p.y)
                                    && Double.isFinite(p.z)) {
                                telemetry.addData(
                                        "Tag " + id + " XYZ camera (in)",
                                        "X=%.1f Y=%.1f Z=%.1f",
                                        p.x, p.y, p.z
                                );
                            }
                        }
                    }
                }

                // Field XYZ requires the camera at its calibrated mount angle.
                if ("OK".equals(status)
                        && result.getBotposeTagCount() > 0) {
                    Pose3D robotPose = result.getBotpose();
                    if (robotPose != null
                            && robotPose.getPosition() != null) {
                        Position p = robotPose.getPosition()
                                .toUnit(DistanceUnit.INCH);

                        if (Double.isFinite(p.x)
                                && Double.isFinite(p.y)
                                && Double.isFinite(p.z)) {
                            telemetry.addData(
                                    "Field -> robot XYZ (in)",
                                    "X=%.1f Y=%.1f Z=%.1f",
                                    p.x, p.y, p.z
                            );
                        }
                    }
                }

                telemetry.update();
                idle();
            }
        } finally {
            camera.stop();
        }
    }
}