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

    //private HardwareMap hw;
    private Telemetry telemetry;
    private Gamepad gamepad1, gamepad2;

    // Switches
    public boolean autoShootIsActive = false;
    private boolean xButtonWasPressed = false;

    // Constants
    private final double offsetCamToShooter = 6.0; // in inches

    public Robot(HardwareMap hw, Telemetry telemetry, Gamepad gamepad1, Gamepad gamepad2) {
//        this.hw = hw;
        this.telemetry = telemetry;
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;

        this.drivetrain = new Drivetrain(hw, telemetry, gamepad1);
        this.shooter = new Shooter(hw, telemetry, gamepad2);
        this.tube = new Tube(hw, telemetry, gamepad2);
        this.vision = new VisionSystem(hw, telemetry);
    }

    public void handleIntakeOuttake(boolean telemetryActive) {
        if (this.gamepad2.left_trigger > 0.5) {
            this._intake();
        } else if (this.gamepad2.right_trigger > 0.5) {
            this._outtake();
        } else {
            this.shooter.stopShooter();
            this.tube.stop();
        }

        if (telemetryActive) {
            telemetry.addLine("Shooter Power: " + this.shooter.getOutTakePower());
        }

    }

    public void handleAutoShoot(double basketHeight, String allianceColor) {

        // Toggle auto-shoot with proper debouncing
        if (this.gamepad2.x && !xButtonWasPressed) {
            this.autoShootIsActive = !this.autoShootIsActive;
            xButtonWasPressed = true;
        } else if (!this.gamepad2.x) {
            xButtonWasPressed = false;
        }


        if (this.vision.getClosestTag() != null && this.autoShootIsActive) {
            AprilTag tag = this.vision.getTagFromDetection();

            // If not Color Tag and Not Alliance Tag then return
            if (!tag.type.equals(AprilTag.TagType.COLOR) || !tag.color.equals(allianceColor)) return;

            // Center Robot with Basket
            double x = tag.x + this.offsetCamToShooter;
            double buffer = 2; // Amount that the x can be off by. This helps prevent the robot from jittering back and forth.
            double turnSpeed = 0.2;

            if (x > buffer) {
                this.drivetrain.turnLeft(turnSpeed);
            } else if (x < -buffer) {
                this.drivetrain.turnRight(turnSpeed);
            } else {
                this.drivetrain.stop();
            }

            //tag.printTelemetry(telemetry);
            double distance = tag.y;

            this.shooter.goToOptimalShootingAngleAndSpeed(distance, basketHeight);
        }
    }

    private void _intake() {
        this.telemetry.addLine("InTaking...");
        this.shooter.inTake();
        this.tube.inTake();
    }

    private void _outtake() {
        this.telemetry.addLine("OutTaking...");

        this.shooter.outTake();
        if (this.shooter.readyToOutTake()) {
            this.tube.outTake();
        }
    }

}
