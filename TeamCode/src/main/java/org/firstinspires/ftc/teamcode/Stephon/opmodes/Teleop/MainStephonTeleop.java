package org.firstinspires.ftc.teamcode.Stephon.opmodes.Teleop;

import com.pedropathing.telemetry.SelectableOpMode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Stephon.Robot;
import org.firstinspires.ftc.teamcode.Stephon.subsystems.VisionSystem;
import org.firstinspires.ftc.teamcode.Stephon.util.AprilTag;


@TeleOp(name = "Main TeleOp (Select Alliance)", group = "Stephon")
public class MainStephonTeleop extends SelectableOpMode {

    public MainStephonTeleop() {
        super("Select Alliance", s -> {
            s.add("🔴 RED Alliance", () -> new AllianceTeleop("red"));
            s.add("🔵 BLUE Alliance", () -> new AllianceTeleop("blue"));
        });
    }

    @Override
    public void onSelect() {
        // Called when selection is made - can initialize shared resources here if needed
    }
}

class AllianceTeleop extends OpMode {
    private Robot stephon;

    // Constants
    private static final double BASKET_HEIGHT = 53;
    private final String allianceColor;

    public AllianceTeleop(String allianceColor) {
        this.allianceColor = allianceColor;
    }

    @Override
    public void init() {
        stephon = new Robot(hardwareMap, telemetry, gamepad1, gamepad2);
    }

    @Override
    public void loop() {
        stephon.drivetrain.driveControl();

        stephon.handleAutoShoot(BASKET_HEIGHT, allianceColor);
        if (!stephon.autoShootIsActive) stephon.shooter.angleControl(false);
        stephon.handleIntakeOuttake(false);

        telemetry.update();

    }
}
