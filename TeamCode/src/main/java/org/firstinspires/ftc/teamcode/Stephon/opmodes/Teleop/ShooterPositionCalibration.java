package org.firstinspires.ftc.teamcode.Stephon.opmodes.Teleop;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.telemetry.SelectableOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Stephon.Robot;
import org.firstinspires.ftc.teamcode.Stephon.opmodes.Autonomous.pedroPathing.Constants;

/**
 * Shooter Position Calibration TeleOp
 *
 * This TeleOp allows testing and calibration of shooter settings at different positions.
 *
 * Controls:
 * - DPAD UP/DOWN: Adjust tilt angle
 * - DPAD LEFT/RIGHT: Adjust motor speed
 * - A: Move to selected position
 * - B: Stop movement and hold position
 *
 * Use this to find optimal tilt angle and motor speed for each shooting position.
 */
@TeleOp(name = "Shooter Position Calibration", group = "Calibration")
public class ShooterPositionCalibration extends SelectableOpMode {

    public ShooterPositionCalibration() {
        super("Select Position", s -> {
            s.add("PreLoad Position", PreLoadCalibration::new);
            s.add("Row 1 Shoot Position", Row1Calibration::new);
            s.add("Row 2 Shoot Position", Row2Calibration::new);
            s.add("Row 3 Shoot Position", Row3Calibration::new);
        });
    }
}

abstract class PositionCalibration extends OpMode {
    protected Robot robot;
    protected Follower follower;
    protected PathChain targetPath;
    protected boolean isMoving = false;

    // Calibration variables
    protected double currentMotorSpeed = 0.4;
    protected final double SPEED_INCREMENT = 0.01;

    @Override
    public void init() {
        robot = new Robot(hardwareMap, telemetry, gamepad1, gamepad2);
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(63.01182654402103, 9.5, Math.toRadians(90)));

        // Initialize shooter
        robot.shooter.stopShooter();
        robot.tube.stop();

        telemetry.addLine("Shooter Position Calibration Initialized");
        telemetry.addLine("Left Stick Y: Adjust Tilt Angle");
        telemetry.addLine("DPAD LEFT/RIGHT: Adjust Motor Speed");
        telemetry.addLine("X: Move to position / Start shooting");
        telemetry.addLine("B: Stop / Hold position");
        telemetry.update();
    }

    @Override
    public void loop() {
        // Update follower
        follower.update();
        robot.vision.telemetryTags();

        // Angle control using left joystick (handles both left and right servo automatically)
        robot.shooter.angleControl(false);

        // Handle motor speed adjustment (DPAD LEFT/RIGHT)
        if (gamepad2.dpad_right) {
            currentMotorSpeed = Math.min(1.0, currentMotorSpeed + SPEED_INCREMENT);
        } else if (gamepad2.dpad_left) {
            currentMotorSpeed = Math.max(0.0, currentMotorSpeed - SPEED_INCREMENT);
        }

        // Move to position (X button)
        if (gamepad2.x) {
            if (!isMoving) {
                follower.followPath(targetPath);
                isMoving = true;
            }
            // Start shooting when at position
            if (!follower.isBusy()) {
                robot.shooter.setOutTakePower(currentMotorSpeed);
                robot.shooter.outTake();
                robot.tube.outTake();
            }
        }

        // Stop movement (B button)
        if (gamepad2.b) {
            isMoving = false;
            robot.shooter.stopShooter();
            robot.tube.stop();
            // Hold current position
            if (!follower.isBusy()) {
                follower.holdPoint(follower.getPose());
            }
        }

        // Display calibration values
        telemetry.addLine("========== CALIBRATION VALUES ==========");
        telemetry.addLine(String.format("Motor Speed: %.3f", currentMotorSpeed));
        telemetry.addLine("");
        telemetry.addLine("Position: " + getPositionName());
        telemetry.addLine("Follower Busy: " + follower.isBusy());
        telemetry.addLine("Is Moving: " + isMoving);
        telemetry.addLine("");
        telemetry.addLine("Log these values when shot accuracy is good:");
        telemetry.addLine("Position: " + getPositionName());
        telemetry.addLine("Speed: " + String.format("%.3f", currentMotorSpeed));
        telemetry.update();
    }

    protected abstract String getPositionName();
}

class PreLoadCalibration extends PositionCalibration {
    @Override
    public void init() {
        super.init();
        // Simple straight line to basket for preload
        targetPath = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(63.012, 9.500), new Pose(63.000, 14.949))
                )
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(113.5))
                .build();

        currentMotorSpeed = 0.8;
    }

    @Override
    protected String getPositionName() {
        return "PreLoad";
    }
}

class Row1Calibration extends PositionCalibration {
    @Override
    public void init() {
        super.init();
        // Straight line to row 1 shooting position
        targetPath = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(63.012, 9.500), new Pose(63.000, 14.949))
                )
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(113.5))
                .build();

        currentMotorSpeed = 0.75;
    }

    @Override
    protected String getPositionName() {
        return "Row 1";
    }
}

class Row2Calibration extends PositionCalibration {
    @Override
    public void init() {
        super.init();
        // Straight line to row 2 shooting position
        targetPath = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(63.012, 9.500), new Pose(69.256, 75.122))
                )
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(135.5))
                .build();

        currentMotorSpeed = 0.80;
    }

    @Override
    protected String getPositionName() {
        return "Row 2";
    }
}

class Row3Calibration extends PositionCalibration {
    @Override
    public void init() {
        super.init();
        // Straight line to row 3 shooting position
        targetPath = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(63.012, 9.500), new Pose(69.256, 75.122))
                )
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(135.5))
                .build();

        currentMotorSpeed = 0.85;
    }

    @Override
    protected String getPositionName() {
        return "Row 3";
    }
}