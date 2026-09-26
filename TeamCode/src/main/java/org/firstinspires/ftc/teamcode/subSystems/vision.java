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
public class vision {
// TODO: Make sure that blue/red alliance are differentiable
    // ------ Settings -----
    public static final class Config {
        // Name in the Robot Configs
        public static final String HARDWARE_NAME = "vision";

        // TODO: set to the real AprilTag pipeline number
        public static final int APRILTAG_PIPELINE_INDEX = 0;

        // How often to ask LL for data
        public static final int POLL_RATE_HZ = 100;

        // TODO: tune. Data older than this is ignored
        public static final long MAX_STALENESS_MS = 100;

        public static final String EXPECTED_TAG_FAMILY = "36h11";

        // CELL tag IDs for each alliance
        public static final int RED_CELL_ID_MIN = 30;
        public static final int RED_CELL_ID_MAX = 37;
        public static final int BLUE_CELL_ID_MIN = 38;
        public static final int BLUE_CELL_ID_MAX = 45;

        private Config() { }
    }

    public enum Alliance { RED, BLUE, NONE }

    // Basically whether or not a frame can be used. Only safe to aim with.
    public enum Status {
        NOT_STARTED,
        DISCONNECTED,
        NO_RESULT,
        INVALID_RESULT,
        WRONG_PIPELINE,
        STALE,
        OK
    }

    // In the instance that only one tag seen by the camera
    public static final class TagObservation {
        public final int id;
        public final Alliance alliance;  // NONE if not a CELL tag
        public final double txDeg;       // left/right angle to tag
        public final double tyDeg;       // up/down angle to tag
        public final double area;        // tag size in image (last year's range table used this)

        public final boolean hasPose;    // false if "Full 3D" is off
        public final double camXIn, camYIn, camZIn;    // tag position from camera, inches
        public final double rangeIn;                   // distance to tag, inches
        public final double yawDeg, pitchDeg, rollDeg; // tag rotation from camera

        TagObservation(int id, Alliance alliance, double txDeg, double tyDeg, double area,
                       boolean hasPose, double camXIn, double camYIn, double camZIn,
                       double rangeIn, double yawDeg, double pitchDeg, double rollDeg) {
            this.id = id;
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

    // A tag we threw out, and why
    public static final class RejectedTag {
        public final int id;
        public final String reason;

        RejectedTag(int id, String reason) {
            this.id = id;
            this.reason = reason;
        }
    }

    // Everything from one camera read
    public static final class Frame {
        public final Status status;
        public final int reportedPipelineIndex; // -1 if unknown
        public final long stalenessMs;          // age of the data, -1 if unknown
        public final double readTimeSec;        // when we read it
        public final List<TagObservation> tags;
        public final List<RejectedTag> rejected;

        Frame(Status status, int reportedPipelineIndex, long stalenessMs, double readTimeSec,
              List<TagObservation> tags, List<RejectedTag> rejected) {
            this.status = status;
            this.reportedPipelineIndex = reportedPipelineIndex;
            this.stalenessMs = stalenessMs;
            this.readTimeSec = readTimeSec;
            this.tags = Collections.unmodifiableList(tags);
            this.rejected = Collections.unmodifiableList(rejected);
        }

        // Only aim when this is true
        public boolean isUsable() {
            return status == Status.OK;
        }

        static Frame empty(Status status, int pipeline, long stalenessMs, double readTimeSec) {
            return new Frame(status, pipeline, stalenessMs, readTimeSec,
                    new ArrayList<TagObservation>(), new ArrayList<RejectedTag>());
        }
    }

    private final Limelight3A limelight;
    private final ElapsedTime clock = new ElapsedTime();
    private boolean started = false;
    private Frame latestFrame;

    // ----- LL Setup -----

    public vision(HardwareMap hardwareMap) {
        try {
            limelight = hardwareMap.get(Limelight3A.class, Config.HARDWARE_NAME);
        } catch (IllegalArgumentException e) {
            throw new IllegalStateException(
                    "Limelight3A not found in Robot Configuration under name \""
                            + Config.HARDWARE_NAME + "\"", e);
        }
        limelight.setPollRateHz(Config.POLL_RATE_HZ); // must be before start()
        limelight.pipelineSwitch(Config.APRILTAG_PIPELINE_INDEX);
        latestFrame = Frame.empty(Status.NOT_STARTED, -1, -1, clock.seconds());
    }

    public void start() {
        limelight.start();
        started = true;
    }

    // Call when the OpMode ends
    public void stop() {
        limelight.stop();
        started = false;
        latestFrame = Frame.empty(Status.NOT_STARTED, -1, -1, clock.seconds());
    }

    // Read the camera once. Call this method once per loop.
    public Frame update() {
        double now = clock.seconds();

        if (!started) {
            latestFrame = Frame.empty(Status.NOT_STARTED, -1, -1, now);
            return latestFrame;
        }
        if (!limelight.isConnected()) {
            latestFrame = Frame.empty(Status.DISCONNECTED, -1, -1, now);
            return latestFrame;
        }

        LLResult result = limelight.getLatestResult();
        if (result == null) {
            latestFrame = Frame.empty(Status.NO_RESULT, -1, -1, now);
            return latestFrame;
        }

        int pipeline = result.getPipelineIndex();
        long stalenessMs = result.getStaleness();

        if (!result.isValid()) {
            latestFrame = Frame.empty(Status.INVALID_RESULT, pipeline, stalenessMs, now);
            return latestFrame;
        }
        if (pipeline != Config.APRILTAG_PIPELINE_INDEX) {
            latestFrame = Frame.empty(Status.WRONG_PIPELINE, pipeline, stalenessMs, now);
            return latestFrame;
        }
        if (stalenessMs < 0 || stalenessMs > Config.MAX_STALENESS_MS) {
            latestFrame = Frame.empty(Status.STALE, pipeline, stalenessMs, now);
            return latestFrame;
        }

        List<TagObservation> tags = new ArrayList<>();
        List<RejectedTag> rejected = new ArrayList<>();

        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
        if (fiducials != null) {
            for (LLResultTypes.FiducialResult fr : fiducials) {
                convert(fr, tags, rejected);
            }
        }

        latestFrame = new Frame(Status.OK, pipeline, stalenessMs, now, tags, rejected);
        return latestFrame;
    }

    // Last frame from update(). Would never be null.
    public Frame getLatestFrame() {
        return latestFrame;
    }

    // Turn one Limelight tag into a TagObservation (or reject it if not applicable)
    private static void convert(LLResultTypes.FiducialResult fr,
                                List<TagObservation> tags, List<RejectedTag> rejected) {
        if (fr == null) {
            return;
        }
        int id = fr.getFiducialId();

        if (!familyMatches(fr.getFamily())) {
            rejected.add(new RejectedTag(id, "family " + fr.getFamily()));
            return;
        }

        double txDeg = fr.getTargetXDegrees();
        double tyDeg = fr.getTargetYDegrees();
        if (!Double.isFinite(txDeg) || !Double.isFinite(tyDeg)) {
            rejected.add(new RejectedTag(id, "bad tx/ty"));
            return;
        }

        // 3D data only exists if "Full 3D" is on. If missing, range is NaN. CHECK
        double x = Double.NaN, y = Double.NaN, z = Double.NaN, range = Double.NaN;
        double yaw = Double.NaN, pitch = Double.NaN, roll = Double.NaN;
        boolean hasPose = false;

        Pose3D pose = fr.getTargetPoseCameraSpace();
        if (pose != null && pose.getPosition() != null && pose.getOrientation() != null) {
            Position p = pose.getPosition().toUnit(DistanceUnit.INCH);
            YawPitchRollAngles o = pose.getOrientation();
            double px = p.x, py = p.y, pz = p.z;
            double r = Math.sqrt(px * px + py * py + pz * pz);
            double pyaw = o.getYaw(AngleUnit.DEGREES);
            double ppitch = o.getPitch(AngleUnit.DEGREES);
            double proll = o.getRoll(AngleUnit.DEGREES);

            // Note: All zeros means no 3D data
            boolean finite = Double.isFinite(r) && Double.isFinite(pyaw)
                    && Double.isFinite(ppitch) && Double.isFinite(proll);
            if (finite && r > 0.0) {
                hasPose = true;
                x = px; y = py; z = pz; range = r;
                yaw = pyaw; pitch = ppitch; roll = proll;
            }
        }

        double area = fr.getTargetArea();
        if (!Double.isFinite(area) || area < 0.0) {
            area = Double.NaN;
        }

        tags.add(new TagObservation(id, allianceForId(id), txDeg, tyDeg, area,
                hasPose, x, y, z, range, yaw, pitch, roll));
    }

    // Accepts "36h11", "36H11", "tag36h11". Blank is allowed.
    private static boolean familyMatches(String family) {
        if (family == null || family.isEmpty()) {
            return true;
        }
        String norm = family.toLowerCase().replaceAll("[^a-z0-9]", "");
        return norm.endsWith(Config.EXPECTED_TAG_FAMILY);
    }

    // Which alliance a tag ID belongs to
    public static Alliance allianceForId(int id) {
        if (id >= Config.RED_CELL_ID_MIN && id <= Config.RED_CELL_ID_MAX) {
            return Alliance.RED;
        }
        if (id >= Config.BLUE_CELL_ID_MIN && id <= Config.BLUE_CELL_ID_MAX) {
            return Alliance.BLUE;
        }
        return Alliance.NONE;
    }

    // ---- Telemetry ----

    public void addTelemetry(Telemetry telemetry) {
        Frame f = latestFrame;
        telemetry.addLine("--- Vision (Limelight) ---");
        telemetry.addData("LL", "connected=%b running=%b",
                limelight.isConnected(), limelight.isRunning());

        LLStatus s = limelight.getStatus();
        if (s != null) {
            telemetry.addData("LL status", "pipe %d (%s)  fps %d  temp %.1fC  cpu %.0f%%",
                    s.getPipelineIndex(), s.getPipelineType(),
                    (int) s.getFps(), s.getTemp(), s.getCpu());
        }

        telemetry.addData("Frame", "%s  pipe=%d (want %d)  age=%dms",
                f.status, f.reportedPipelineIndex, Config.APRILTAG_PIPELINE_INDEX, f.stalenessMs);

        if (f.tags.isEmpty()) {
            telemetry.addData("Tags", "none");
        } else {
            StringBuilder ids = new StringBuilder();
            for (TagObservation t : f.tags) {
                if (ids.length() > 0) ids.append(' ');
                ids.append(t.id);
            }
            telemetry.addData("Tags (" + f.tags.size() + ")", ids.toString());
            for (TagObservation t : f.tags) {
                telemetry.addData("  tag " + t.id,
                        "%s tx=%.1f ty=%.1f ta=%.4f range=%s",
                        t.alliance, t.txDeg, t.tyDeg, t.area,
                        t.hasPose ? String.format("%.1fin", t.rangeIn) : "n/a");
            }
        }

        for (RejectedTag r : f.rejected) {
            telemetry.addData("  rejected " + r.id, r.reason);
        }
    }
}