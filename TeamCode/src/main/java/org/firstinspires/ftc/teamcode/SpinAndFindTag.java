package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;


@TeleOp
public class SpinAndFindTag extends LinearOpMode {

    double x;
    double y;
    double z;
    double pitch;
    double roll;
    double yaw;
    double range;
    double elevation;
    double bearing;

    private DcMotor leftFront = null;
    private DcMotor leftBack = null;
    private DcMotor rightFront = null;
    private DcMotor rightBack = null;

    boolean doSpin = true;

    @Override
    public void runOpMode() throws InterruptedException {

        leftFront = hardwareMap.get(DcMotor.class, "leftfront");
        leftBack = hardwareMap.get(DcMotor.class, "leftback");
        rightFront = hardwareMap.get(DcMotor.class, "rightfront");
        rightBack = hardwareMap.get(DcMotor.class, "rightback");

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.FORWARD);

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
                    doSpin = false;
                    x = tag.ftcPose.x;
                    y = tag.ftcPose.y;
                    z = tag.ftcPose.z;
                    pitch = tag.ftcPose.pitch;
                    roll = tag.ftcPose.roll;
                    yaw = tag.ftcPose.yaw;
                    range = tag.ftcPose.range;
                    elevation = tag.ftcPose.elevation;
                    bearing = tag.ftcPose.bearing;

                    if (x > 0.0) {
                        leftFront.setPower(0.3);
                        leftBack.setPower(0.3);
                    } else if (x < 0.0) {
                        rightFront.setPower(0.3);
                        rightBack.setPower(0.3);
                    } else {
                        leftFront.setPower(0.0);
                        leftBack.setPower(0.0);
                        rightFront.setPower(0.0);
                        rightBack.setPower(0.0);
                    }

                    telemetry.addData("x", x);
                    telemetry.addData("y", y);
                    telemetry.addData("z", z);
                } else {
                    telemetry.addLine("No FTC Pose available (tag not in field layout or bad detection)");
                }
            } else {

                if (doSpin) {
                    leftFront.setPower(0.3);
                    leftBack.setPower(0.3);
                    rightFront.setPower(-0.3);
                    rightBack.setPower(-0.3);

                } else {

                    leftFront.setPower(0.0);
                    leftBack.setPower(0.0);
                    rightFront.setPower(0.0);
                    rightBack.setPower(0.0);
                }
                telemetry.addLine("No tags detected");
            }


            telemetry.update();

        }
    }

}
