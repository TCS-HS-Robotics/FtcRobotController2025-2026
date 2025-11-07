package org.firstinspires.ftc.teamcode.Stephon.subsystems;

import android.util.Size;
import androidx.annotation.Nullable;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Stephon.util.AprilTag;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.List;

public class VisionSystem {

    // --- Known Tags in the Environment ---
    private final List<AprilTag> knownTags = new ArrayList<>();

    // --- Camera and Vision Components ---
    private final WebcamName camera;
    private final AprilTagProcessor tagProcessor;
    private final VisionPortal visionPortal;

    Telemetry telemetry;

    // --- Constructor ---
    public VisionSystem(HardwareMap hw, Telemetry telemetry) {
        this.telemetry = telemetry;

        // Initialize hardware and processors
        camera = hw.get(WebcamName.class, "Webcam 1");

        tagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .build();

        visionPortal = new VisionPortal.Builder()
                .addProcessor(tagProcessor)
                .setCamera(camera)
                .setCameraResolution(new Size(640, 480))
                .build();
    }

    // --- Periodic Update (called every loop) ---
    public void mainLoop(@Nullable Gamepad gamepad) {
        // You can later use the gamepad to switch pipelines or modes.
        //AprilTag.handleTag(tagProcessor, knownTags, telemetry);
    }

    // --- Get all detections from FTC SDK ---
    public ArrayList<AprilTagDetection> getDetections() {
        return tagProcessor.getDetections();
    }

    // --- Get the closest tag by Euclidean distance ---
    public @Nullable AprilTagDetection getClosestTag() {
        ArrayList<AprilTagDetection> detections = getDetections();
        if (detections.isEmpty()) return null;

        AprilTagDetection closest = detections.get(0);
        double minDist = AprilTag.getTagDistance(closest);

        for (AprilTagDetection d : detections) {
            double dist = AprilTag.getTagDistance(d);
            if (dist < minDist) {
                closest = d;
                minDist = dist;
            }
        }
        return closest;
    }

    // --- Get the first target tag ---
    public @Nullable AprilTag getFirstTargetTag() {
        ArrayList<AprilTagDetection> detections = getDetections();
        if (detections.isEmpty()) return null;

        for (AprilTagDetection detection : detections) {
            if (AprilTag.getTagTypeFromDetection(detection).equals("Color")) {
                return AprilTag.fromDetection(detection);
            }
        }
        return null;
    }

    // -- Get the data from tag ---
    public AprilTag getTagFromDetection() {
        return AprilTag.fromDetection(getClosestTag());
    }

    // --- Lookup known tag by ID ---
    public @Nullable AprilTag getTagById(int id) {
        for (AprilTag tag : knownTags) {
            if (tag.id == id) return tag;
        }
        return null;
    }

    // --- Display all tag info to telemetry ---
    public void telemetryTags() {
        ArrayList<AprilTagDetection> detections = getDetections();

        if (detections.isEmpty()) {
            telemetry.addLine("No AprilTags detected.");
            return;
        }

        for (AprilTagDetection detection : detections) {
            telemetry.addLine("Detected ID: " + detection.id);
            telemetry.addLine(String.format("X: %.1f  Y: %.1f  Z: %.1f",
                    detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));

            AprilTag known = getTagById(detection.id);
            if (known != null)
                telemetry.addLine("Known Tag Info:\n" + known.toString());

            telemetry.addLine("Distance (cm): " + AprilTag.getTagDistance(detection));
            telemetry.addLine("Goal Dist (cm): " + AprilTag.getGoalDistance(detection));
            telemetry.addLine("--------------------------------");
        }
    }
}
