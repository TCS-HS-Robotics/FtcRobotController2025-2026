package org.firstinspires.ftc.teamcode.Stephon.opmodes.Teleop;

import com.pedropathing.telemetry.SelectableOpMode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Stephon.Robot;
import org.firstinspires.ftc.teamcode.Stephon.subsystems.VisionSystem;
import org.firstinspires.ftc.teamcode.Stephon.util.AprilTag;


@TeleOp(name="Main TeleOp (Select Alliance)", group="Stephon")
public class MainStephonTeleop extends LinearOpMode {


    private Robot robot;

    // --- Constants ---
    private final double BASKET_HEIGHT = 53;

    @Override
    public void runOpMode() {

        robot = new Robot(hardwareMap, telemetry, gamepad1, gamepad2);

        boolean autoShoot = false;

        waitForStart();

        while (opModeIsActive()) {
            //robot.mainLoop(gamepad1, gamepad2);

            if (gamepad2.x) {
                autoShoot = !autoShoot;
            }

            // If tag is detected, go to optimal shooting angle
            if (robot.vision.getClosestTag() != null && autoShoot) {
                AprilTag tag = robot.vision.getTagFromDetection();
                tag.printTelemetry(telemetry);
                double distance = tag.y;

                robot.shooter.goToOptimalShootingAngleAndSpeed(gamepad2, distance, BASKET_HEIGHT);
            }

            robot.drivetrain.driveControl(gamepad1);
            robot.shooter.angleControl(gamepad2);

            if (gamepad2.left_trigger > 0.5) {
                robot.intake();
            } else if (gamepad2.right_trigger > 0.5) {
                robot.outtake();
            } else {
                robot.shooter.stopShooter();
                robot.tube.stop();
            }
        }

        telemetry.update();

    }

}