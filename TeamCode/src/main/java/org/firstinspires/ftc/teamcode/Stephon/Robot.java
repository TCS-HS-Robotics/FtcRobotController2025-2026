package org.firstinspires.ftc.teamcode.Stephon;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Stephon.hardware.Drivetrain;
import org.firstinspires.ftc.teamcode.Stephon.hardware.Shooter;
import org.firstinspires.ftc.teamcode.Stephon.hardware.Tube;
import org.firstinspires.ftc.teamcode.Stephon.subsystems.VisionSystem;
import org.firstinspires.ftc.teamcode.Stephon.util.AprilTag;

public class Robot {

    public Drivetrain drivetrain;
    public Shooter shooter;
    public Tube tube;
    public VisionSystem vision;

    private HardwareMap hw;
    private Telemetry telemetry;
    private Gamepad gamepad1, gamepad2;

    public boolean autoShootIsActive = false;

    public Robot(HardwareMap hw, Telemetry telemetry, Gamepad gamepad1, Gamepad gamepad2) {
        this.hw = hw;
        this.telemetry = telemetry;
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;

        this.drivetrain = new Drivetrain(hw, telemetry);
        this.shooter = new Shooter(hw, telemetry);
        this.tube = new Tube(hw, telemetry);
        this.vision = new VisionSystem(hw, telemetry);
    }

    public void mainLoop(Gamepad gamepad1, Gamepad gamepad2) {
        this.drivetrain.mainLoop(gamepad1);
        this.shooter.mainLoop(gamepad2);
        this.tube.mainLoop(null);
        this.vision.mainLoop(null);
    }

    public void intake() {
        this.telemetry.addLine("InTaking...");
        this.shooter.inTake();
        this.tube.inTake();
    }

    public void outtake() {
        this.telemetry.addLine("OutTaking...");

        this.shooter.outTake();
        if (this.shooter.readyToOutTake()) {
            this.tube.outTake();
        }
    }

    public void handleAutoShoot(double basketHeight) {
        if (this.vision.getClosestTag() != null && autoShootIsActive) {
            AprilTag tag = this.vision.getTagFromDetection();
            //tag.printTelemetry(telemetry);
            double distance = tag.y;

            this.shooter.goToOptimalShootingAngleAndSpeed(gamepad2, distance, basketHeight);
        }
    }

}
