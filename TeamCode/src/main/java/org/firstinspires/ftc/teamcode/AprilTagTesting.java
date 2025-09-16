package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;


@TeleOp
public class AprilTagTesting extends LinearOpMode {

    double x;
    double y;
    double z;
    double pitch;
    double roll;
    double yaw;
    double range;
    double elevation;
    double bearing;

    @Override
    public void runOpMode() throws InterruptedException {

        // Create Tag Processor
        AprilTagProcessor tagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(true) // Draws the axis on the tag
                .setDrawCubeProjection(true) // Draws a cube projection on the tag
                .setDrawTagID(true) // Draws the tag number on the tag
                .setDrawTagOutline(true) // Draws the tag outline on the tag
                .build();

        // This is necessary for having the camera stream on the Driver Station
        VisionPortal visionPortal = new VisionPortal.Builder()
                .addProcessor(tagProcessor)
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(640, 480))
                .build();

        waitForStart();

        // opMode actions
        while(!isStopRequested() && opModeIsActive()) {

            // Check if a tag is detected
            if (!tagProcessor.getDetections().isEmpty()) {
                // Get the first tag found
                AprilTagDetection tag = tagProcessor.getDetections().get(0);

                telemetry.addData("Tag ID", tag.id);

                // Make sure ftcPose is not null before using it
                if (tag.ftcPose != null) {
                    x = tag.ftcPose.x;
                    y = tag.ftcPose.y;
                    z = tag.ftcPose.z;
                    pitch = tag.ftcPose.pitch;
                    roll = tag.ftcPose.roll;
                    yaw = tag.ftcPose.yaw;
                    range = tag.ftcPose.range;
                    elevation = tag.ftcPose.elevation;
                    bearing = tag.ftcPose.bearing;

                    telemetry.addData("x", x);
                    telemetry.addData("y", y);
                    telemetry.addData("z", z);
                    telemetry.addData("pitch", pitch);
                    telemetry.addData("roll", roll);
                    telemetry.addData("yaw", yaw);
                    telemetry.addData("range", range);
                    telemetry.addData("elevation", elevation);
                    telemetry.addData("bearing", bearing);
                } else {
                    telemetry.addLine("No FTC Pose available (tag not in field layout or bad detection)");
                }
            } else {
                telemetry.addLine("No tags detected");
            }


            telemetry.update();

        }
    }

}
