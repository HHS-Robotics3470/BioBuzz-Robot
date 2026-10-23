package org.firstinspires.ftc.teamcode.subSystems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

public class Vision {
    public static final class Config {
        public static final String HARDWARE_NAME = "vision";
        public static final int APRILTAG_PIPELINE_INDEX = 0;
        public static final int POLL_RATE_HZ = 100;
        public static final long MAX_STALENESS_MS = 250;

        public static final int RED_CELL_ID_MIN = 30;
        public static final int RED_CELL_ID_MAX = 37;
        public static final int BLUE_CELL_ID_MIN = 38;
        public static final int BLUE_CELL_ID_MAX = 45;

        private Config() { }
    }

    public enum Alliance {
        RED, BLUE, NONE
    }

    public enum Status {
        NOT_STARTED,
        DISCONNECTED,
        NO_RESULT,
        INVALID_RESULT,
        WRONG_PIPELINE,
        STALE,
        OK
    }

    public static final class TagObservation {
        public final int id;
        public final String family;
        public final Alliance alliance;
        public final double txDeg;
        public final double tyDeg;
        public final double area;

        public final boolean hasPose;
        public final double camXIn;
        public final double camYIn;
        public final double camZIn;
        public final double rangeIn;
        public final double yawDeg;
        public final double pitchDeg;
        public final double rollDeg;

        TagObservation(int id, String family, Alliance alliance,
                       double txDeg, double tyDeg, double area,
                       boolean hasPose, double camXIn, double camYIn,
                       double camZIn, double rangeIn, double yawDeg,
                       double pitchDeg, double rollDeg) {
            this.id = id;
            this.family = family;
            this.alliance = alliance;
            this.txDeg = txDeg;
            this.tyDeg = tyDeg;
            this.area = area;
            this.hasPose = hasPose;
            this.camXIn = camXIn;
            this.camYIn = camYIn;
            this.camZIn = camZIn;
            this.rangeIn = rangeIn;
            this.yawDeg = yawDeg;
            this.pitchDeg = pitchDeg;
            this.rollDeg = rollDeg;
        }
    }

    public static final class RejectedTag {
        public final int id;
        public final String reason;

        RejectedTag(int id, String reason) {
            this.id = id;
            this.reason = reason;
        }
    }

    public static final class Frame {
        public final Status status;
        public final int reportedPipelineIndex;
        public final long stalenessMs;
        public final double readTimeSec;
        public final List<TagObservation> tags;
        public final List<RejectedTag> rejected;

        Frame(Status status, int reportedPipelineIndex, long stalenessMs,
              double readTimeSec, List<TagObservation> tags,
              List<RejectedTag> rejected) {
            this.status = status;
            this.reportedPipelineIndex = reportedPipelineIndex;
            this.stalenessMs = stalenessMs;
            this.readTimeSec = readTimeSec;
            this.tags = Collections.unmodifiableList(tags);
            this.rejected = Collections.unmodifiableList(rejected);
        }

        public boolean isUsable() {
            return status == Status.OK;
        }

        static Frame empty(Status status, int pipeline,
                           long stalenessMs, double readTimeSec) {
            return new Frame(
                    status, pipeline, stalenessMs, readTimeSec,
                    new ArrayList<TagObservation>(),
                    new ArrayList<RejectedTag>()
            );
        }
    }

    private final Limelight3A limelight;
    private final ElapsedTime clock = new ElapsedTime();
    private boolean started = false;
    private Frame latestFrame;

    public Vision(HardwareMap hardwareMap) {
        try {
            limelight = hardwareMap.get(
                    Limelight3A.class,
                    Config.HARDWARE_NAME
            );
        } catch (IllegalArgumentException e) {
            throw new IllegalStateException(
                    "Limelight3A not found in Robot Configuration under name \""
                            + Config.HARDWARE_NAME + "\"",
                    e
            );
        }

        limelight.setPollRateHz(Config.POLL_RATE_HZ);
        limelight.pipelineSwitch(Config.APRILTAG_PIPELINE_INDEX);
        latestFrame = Frame.empty(
                Status.NOT_STARTED, -1, -1, clock.seconds()
        );
    }

    public void start() {
        limelight.start();
        started = true;
    }

    public void stop() {
        limelight.stop();
        started = false;
        latestFrame = Frame.empty(
                Status.NOT_STARTED, -1, -1, clock.seconds()
        );
    }

    public Frame update() {
        double now = clock.seconds();

        if (!started) {
            latestFrame = Frame.empty(
                    Status.NOT_STARTED, -1, -1, now
            );
            return latestFrame;
        }

        if (!limelight.isConnected()) {
            latestFrame = Frame.empty(
                    Status.DISCONNECTED, -1, -1, now
            );
            return latestFrame;
        }

        LLResult result = limelight.getLatestResult();
        if (result == null) {
            latestFrame = Frame.empty(
                    Status.NO_RESULT, -1, -1, now
            );
            return latestFrame;
        }

        int pipeline = result.getPipelineIndex();
        long stalenessMs = result.getStaleness();

        List<TagObservation> tags = new ArrayList<>();
        List<RejectedTag> rejected = new ArrayList<>();

        List<LLResultTypes.FiducialResult> fiducials =
                result.getFiducialResults();

        if (fiducials != null) {
            for (LLResultTypes.FiducialResult fr : fiducials) {
                convert(fr, tags, rejected);
            }
        }

        // Keep detections for telemetry even if the frame cannot drive aiming.
        Status status = !result.isValid()
                ? Status.INVALID_RESULT
                : pipeline != Config.APRILTAG_PIPELINE_INDEX
                  ? Status.WRONG_PIPELINE
                  : stalenessMs < 0
                || stalenessMs > Config.MAX_STALENESS_MS
                    ? Status.STALE
                    : Status.OK;

        latestFrame = new Frame(
                status, pipeline, stalenessMs, now, tags, rejected
        );
        return latestFrame;
    }

    public Frame getLatestFrame() {
        return latestFrame;
    }

    public boolean isConnected() {
        return limelight.isConnected();
    }

    private static void convert(LLResultTypes.FiducialResult fr,
                                List<TagObservation> tags,
                                List<RejectedTag> rejected) {
        if (fr == null) {
            return;
        }

        int id = fr.getFiducialId();
        double txDeg = fr.getTargetXDegrees();
        double tyDeg = fr.getTargetYDegrees();

        if (!Double.isFinite(txDeg) || !Double.isFinite(tyDeg)) {
            rejected.add(new RejectedTag(id, "bad tx/ty"));
            return;
        }

        double x = Double.NaN;
        double y = Double.NaN;
        double z = Double.NaN;
        double range = Double.NaN;
        double yaw = Double.NaN;
        double pitch = Double.NaN;
        double roll = Double.NaN;
        boolean hasPose = false;

        Pose3D pose = fr.getTargetPoseCameraSpace();
        if (pose != null
                && pose.getPosition() != null
                && pose.getOrientation() != null) {

            Position p = pose.getPosition().toUnit(DistanceUnit.INCH);
            YawPitchRollAngles o = pose.getOrientation();

            double px = p.x;
            double py = p.y;
            double pz = p.z;
            double r = Math.sqrt(px * px + py * py + pz * pz);

            double pyaw = o.getYaw(AngleUnit.DEGREES);
            double ppitch = o.getPitch(AngleUnit.DEGREES);
            double proll = o.getRoll(AngleUnit.DEGREES);

            boolean finite = Double.isFinite(r)
                    && Double.isFinite(pyaw)
                    && Double.isFinite(ppitch)
                    && Double.isFinite(proll);

            if (finite && r > 0.0) {
                hasPose = true;
                x = px;
                y = py;
                z = pz;
                range = r;
                yaw = pyaw;
                pitch = ppitch;
                roll = proll;
            }
        }

        double area = fr.getTargetArea();
        if (!Double.isFinite(area) || area < 0.0) {
            area = Double.NaN;
        }

        String family = fr.getFamily() == null
                ? ""
                : fr.getFamily();

        tags.add(new TagObservation(
                id, family, allianceForId(id),
                txDeg, tyDeg, area,
                hasPose, x, y, z, range,
                yaw, pitch, roll
        ));
    }

    public static Alliance allianceForId(int id) {
        if (id >= Config.RED_CELL_ID_MIN
                && id <= Config.RED_CELL_ID_MAX) {
            return Alliance.RED;
        }
        if (id >= Config.BLUE_CELL_ID_MIN
                && id <= Config.BLUE_CELL_ID_MAX) {
            return Alliance.BLUE;
        }
        return Alliance.NONE;
    }

    public void addTelemetry(Telemetry telemetry) {
        Frame f = latestFrame;

        telemetry.addLine("--- Vision (Limelight) ---");
        telemetry.addData("LL", "connected=%b running=%b",
                limelight.isConnected(),
                limelight.isRunning());

        LLStatus s = limelight.getStatus();
        if (s != null) {
            telemetry.addData(
                    "LL status",
                    "pipe %d (%s) fps %d temp %.1fC cpu %.0f%%",
                    s.getPipelineIndex(),
                    s.getPipelineType(),
                    (int) s.getFps(),
                    s.getTemp(),
                    s.getCpu()
            );
        }

        telemetry.addData(
                "Frame",
                "%s pipe=%d (want %d) age=%dms",
                f.status,
                f.reportedPipelineIndex,
                Config.APRILTAG_PIPELINE_INDEX,
                f.stalenessMs
        );

        if (f.tags.isEmpty()) {
            telemetry.addData("Tags", "none");
        } else {
            StringBuilder ids = new StringBuilder();
            for (TagObservation t : f.tags) {
                if (ids.length() > 0) {
                    ids.append(' ');
                }
                ids.append(t.id)
                        .append('(')
                        .append(t.family)
                        .append(')');
            }

            telemetry.addData(
                    "Tags (" + f.tags.size() + ")",
                    ids.toString()
            );

            for (TagObservation t : f.tags) {
                telemetry.addData(
                        "  tag " + t.id,
                        "%s tx=%.1f ty=%.1f ta=%.4f range=%s",
                        t.alliance,
                        t.txDeg,
                        t.tyDeg,
                        t.area,
                        t.hasPose
                                ? String.format("%.1fin", t.rangeIn)
                                : "n/a"
                );
            }
        }

        for (RejectedTag r : f.rejected) {
            telemetry.addData(
                    "  rejected " + r.id,
                    r.reason
            );
        }
    }
}