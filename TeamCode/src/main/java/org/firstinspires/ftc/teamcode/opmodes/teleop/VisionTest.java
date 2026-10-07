package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subSystems.Vision;

/** Telemetry-only Limelight AprilTag test; no motors are commanded. */
@TeleOp(name = "Vision Test", group = "Test")
public class VisionTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        Vision camera = new Vision(hardwareMap);
        telemetry.setMsTransmissionInterval(50);

        telemetry.addLine("Vision test ready. Press Play.");
        telemetry.update();
        waitForStart();
        if (isStopRequested()) return;

        camera.start();
        try {
            while (opModeIsActive()) {
                Vision.Frame frame = camera.update();

                telemetry.addData("Frame", "%s  age=%dms  tags=%d",
                        frame.status, frame.stalenessMs, frame.tags.size());
                telemetry.addData("Pipeline", "%d (expected %d)",
                        frame.reportedPipelineIndex, Vision.Config.APRILTAG_PIPELINE_INDEX);

                if (frame.tags.isEmpty()) {
                    telemetry.addLine("No tags seen");
                }

                for (Vision.TagObservation tag : frame.tags) {
                    telemetry.addLine("--- Tag " + tag.id + " (" + tag.alliance + ") ---");
                    telemetry.addData("Angle (deg)", "tx=%.1f  ty=%.1f", tag.txDeg, tag.tyDeg);

                    if (tag.hasPose) {
                        telemetry.addData("X (in)", "%.1f", tag.camXIn);
                        telemetry.addData("Y (in)", "%.1f", tag.camYIn);
                        telemetry.addData("Z (in)", "%.1f", tag.camZIn);
                        telemetry.addData("Range (in)", "%.1f", tag.rangeIn);
                    } else {
                        telemetry.addLine("No 3D pose. Check Full 3D in the pipeline.");
                    }
                }

                for (Vision.RejectedTag rejected : frame.rejected) {
                    telemetry.addData("Rejected tag " + rejected.id, rejected.reason);
                }
                telemetry.addLine("Move tag left/right and up/down to check axis signs.");
                telemetry.update();
            }
        } finally {
            camera.stop();
        }
    }
}
