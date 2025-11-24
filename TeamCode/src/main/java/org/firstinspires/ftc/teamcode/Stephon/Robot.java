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
    private boolean yButtonWasPressed = false;

    // Shooter timing control
    private long shooterActivationTime = 0; // When shooter motors were turned on
    private boolean tubeHasBeenFired = false; // Track if tube has fired during this cycle
    private final long SHOOTER_SPINUP_DELAY = 750; // 750ms delay before tube can outtake
    private final long SHOOTER_RECOVERY_DELAY = 200; // 200ms delay after shot for motor recovery

    // Shooter mode control
    public enum ShooterMode {
        AUTOMATIC,  // Auto-detect motor speed and fire
        MANUAL      // Driver controls with D-pad
    }
    private ShooterMode shooterMode = ShooterMode.AUTOMATIC; // Default to automatic
    private boolean dpadUpWasPressed = false; // Debounce for D-pad up

    // Constants
    private final double offsetCamToShooter = 7.5; // in inches

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

    public void handleIntake(boolean telemetryActive) {

        // Handle intake
        if (this.gamepad2.left_trigger > 0.5) {
            this._intake();
        }
        // Stop when no triggers pressed
        else {
            this.shooter.stopShooter();
            this.tube.stop();
        }

        if (telemetryActive) {
            telemetry.addLine("Shooter Power: " + this.shooter.getOutTakePower());
            telemetry.addLine("Mode: " + shooterMode);
        }

    }

    public void handleAutoShoot(double basketHeight, String allianceColor, double correction) {

        if (this.gamepad2.y && !yButtonWasPressed) {
            this.autoShootIsActive = !this.autoShootIsActive;
            yButtonWasPressed = true;
        } else if (!this.gamepad2.y) {
            yButtonWasPressed = false;
        }

        if (this.vision.getClosestTag() != null && this.autoShootIsActive) {
            AprilTag tag = this.vision.getTagFromDetection();
            telemetry.addLine("Sees tag");

            // If not Color Tag and Not Alliance Tag then return
            if (!tag.type.equals(AprilTag.TagType.COLOR) || !tag.color.equals(allianceColor)) return;

            // Center Robot with Basket
            //double correction = -1.5; // This is to find the exact number which works
            double x = tag.x + this.offsetCamToShooter + correction;
            double buffer = 1.5; // Amount that the x can be off by. This helps prevent the robot from jittering back and forth.
            double turnSpeed = 0.2;

            if (x > buffer) {
                this.drivetrain.turnRight(turnSpeed);
            } else if (x < -buffer) {
                this.drivetrain.turnLeft(turnSpeed);
            } else {
                this.drivetrain.stop();
            }

            //tag.printTelemetry(telemetry);
            double distance = tag.y;

            this.shooter.goToOptimalShootingAngleAndSpeed(distance, basketHeight);
            telemetry.update();
        } else {
            telemetry.addLine("no tag detected");
            telemetry.update();
        }
    }

//    public void handleIntakeOuttake() {
//        if (this.gamepad2.right_trigger > 0.5) {
//            this.shooter.outTake();
//        } else if (this.gamepad2.left_trigger > 0.5) {
//            this.shooter.inTake();
//            //this.tube.inTake();
//        } else {
//            this.shooter.stopShooter();
//            this.tube.stop();
//        }
//
//    }

    private void _intake() {
        this.telemetry.addLine("InTaking...");
        this.shooter.inTake();
        this.tube.inTake();
    }

    /**
     * AUTOMATIC MODE: Motors spin up, then tube automatically fires after 750ms
     */
//    private void _outtakeAutomatic() {
//        this.shooter.outTake();
//        boolean ready = this.shooter.readyToOutTake();
//        this.telemetry.addLine("Motor Ready: " + ready); // Debug info
//        if (ready) {
//            this.telemetry.addLine("OutTaking (AUTOMATIC)...");
//            this.tube.outTake();
//        }
//    }


    /**
     * MANUAL MODE: Driver controls tube outtake with D-pad Up
     * Motors spin up on right trigger, tube fires when driver presses D-pad Up
     */
//    private void _outtakeManual() {
//        //this.telemetry.addLine("OutTaking (MANUAL)...");
//
//        // Always keep motors running while right trigger is held
//        this.shooter.outTake();
//
//        // Check for D-pad UP to fire the tube
//        if (this.gamepad2.right_bumper) {
//            this.tube.outTake();
//        } else {
//            this.tube.stop();
//        }
//    }

    private void _resetShooterTiming() {
        shooterActivationTime = 0;
        tubeHasBeenFired = false;
    }

}
