package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.UtilClasses.AprilTags.AprilTag;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@TeleOp(name="West Cost Chassy Testing", group="Robot")
public class WestCostChassyTesting extends LinearOpMode {

    DcMotor right;
    DcMotor left;

    double x;

    boolean doSpin = true;

    @Override
    public void runOpMode() throws InterruptedException {

        left = hardwareMap.get(DcMotor.class, "left");
        right = hardwareMap.get(DcMotor.class, "right");

        left.setDirection(DcMotorSimple.Direction.REVERSE);
        right.setDirection(DcMotorSimple.Direction.FORWARD);

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

        while (opModeIsActive()) {
            if (!tagProcessor.getDetections().isEmpty()) {
                AprilTagDetection tag = tagProcessor.getDetections().get(0);

                int id = tag.id;
                telemetry.addLine("Found Tag: " + id);

                if (tag.ftcPose != null) {

                    doSpin = false;
                    x = tag.ftcPose.x;
                    AprilTag.showTagInfo(tagProcessor, telemetry);

                    right.setPower(0.3);
                    left.setPower(0.3);

                } else {
                    telemetry.addLine("Tag has no pose");
                }
            } else {

                if (doSpin) {
                    right.setPower(0.15);
                    left.setPower(-0.15);
                } else {
                    right.setPower(0);
                    left.setPower(0);
                }
                telemetry.addLine("No tags detected");
            }
            telemetry.update();
        }
    }
}
