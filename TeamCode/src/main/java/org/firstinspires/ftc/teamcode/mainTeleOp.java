package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.UtilClasses.AprilTags.AprilTag;
import org.firstinspires.ftc.teamcode.UtilClasses.AprilTags.ColorTag;
import org.firstinspires.ftc.teamcode.UtilClasses.AprilTags.EncodeTag;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;


@TeleOp(name="mainTeleOp", group="Robot")
public class mainTeleOp extends LinearOpMode {

    AprilTag blueTag = new ColorTag(20, "36h11", "blue");
    AprilTag redTag = new ColorTag(24, "36h11", "red");

    AprilTag code1Tag = new EncodeTag(21, "36h11", new char[]{'g', 'p', 'p'});
    AprilTag code2Tag = new EncodeTag(22, "36h11", new char[]{'p', 'g', 'p'});
    AprilTag code3Tag = new EncodeTag(23, "36h11", new char[]{'p', 'p', 'g'});

    ArrayList<AprilTag> tags = new ArrayList<AprilTag>();


    @Override
    public void runOpMode() throws InterruptedException {

        tags.add(blueTag);
        tags.add(redTag);
        tags.add(code1Tag);
        tags.add(code2Tag);
        tags.add(code3Tag);

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

        while (!isStopRequested() && opModeIsActive()) {

            if (!tagProcessor.getDetections().isEmpty()) {

                AprilTag.handleTag(tagProcessor, tags, telemetry);

            } else {
                telemetry.addLine("No tags detected");
            }

            telemetry.update();

        }

    }
}
