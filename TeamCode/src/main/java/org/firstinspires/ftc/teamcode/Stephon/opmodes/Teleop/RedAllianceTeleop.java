package org.firstinspires.ftc.teamcode.Stephon.opmodes.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Stephon.Robot;


@TeleOp(name = "🔴 RED Alliance TeleOp", group = "Stephon")
public class RedAllianceTeleop extends OpMode {
    private Robot stephon;

    // Constants
    private static final double BASKET_HEIGHT = 53;
    private static final String ALLIANCE_COLOR = "red";

    private double autoShooterCorrection = 1.5;


    @Override
    public void init() {
        stephon = new Robot(hardwareMap, telemetry, gamepad1, gamepad2);
    }

    @Override
    public void loop() {
        stephon.drivetrain.driveControl();


        // Toggle auto-shoot with proper debouncing
        stephon.handleAutoShoot(BASKET_HEIGHT, ALLIANCE_COLOR, autoShooterCorrection);

        stephon.shooter.angleControl(false);

        // Shooter intake and outtake. They have to be separate because the shooter needs time to warm up when outtaking so they must be controlled independantly
        //stephon.handleIntakeOuttake();
//        stephon.handleIntake(false);
//        stephon.tube.handleOuttake();
//        stephon.shooter.handleOuttake();
        stephon.shooter.handleIntakeOuttake();
        stephon.tube.handleIntakeOuttake();

        stephon.tube.handleStopper();

        telemetry.update();
    }
}
