package org.firstinspires.ftc.teamcode.UtilClasses.AprilTags;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;

public class AprilTag {

    public int id;
    String family;

    public AprilTag(int id, String family) {
        this.id = id;
        this.family = family;
    }

    @Override
    public String toString() {
        String output = "";
        output += "ID: " + id;
        output += "\nFamily: " + family;
        return output;
    }


    // --- Static Helper Functions ---

    public static void handleTag(AprilTagProcessor tagProcessor, ArrayList<AprilTag> tags, Telemetry telemetry) {
        AprilTagDetection tag = tagProcessor.getDetections().get(0);

        if (tag.ftcPose != null) {

            for (AprilTag t : tags) {
                if (t.id == tag.id) {
                    telemetry.addLine(t.toString());
                    telemetry.addLine("Tag Distance: " + getTagDistance(tag));
                    telemetry.addLine("Goal Distance: " + getGoalDistance(tag));
                }
            }

        } else {
            telemetry.addLine("No FTC Pose available (tag not in field layout or bad detection)");
        }
    }

    public static double getTagDistance(AprilTagDetection detectedTag) {
        double x = detectedTag.ftcPose.x;
        double z = detectedTag.ftcPose.z;

        return Math.sqrt(x*x + z*z);
    }

    public static double getGoalDistance(AprilTagDetection detectedTag) {
        double x = detectedTag.ftcPose.x;
        double y = detectedTag.ftcPose.y;
        double z = detectedTag.ftcPose.z;

        // The center of the goal is about 11.75 inches above the center of the AprilTag. Here its being converted to cm
        double verticalOffset = 11.75 * 0.0254;

        return Math.sqrt(x*x + z*z + Math.pow(y + verticalOffset, 2));
    }

}
