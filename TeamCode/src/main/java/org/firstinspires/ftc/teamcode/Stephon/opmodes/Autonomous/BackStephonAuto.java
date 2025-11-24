package org.firstinspires.ftc.teamcode.Stephon.opmodes.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Stephon.Robot;
import org.firstinspires.ftc.teamcode.Stephon.hardware.Shooter;

@Autonomous(name = "Back Stephon Auto", group = "Stephon")
public class BackStephonAuto extends LinearOpMode {

    private Robot robot;
    private ElapsedTime runtime = new ElapsedTime();
    private final double BASKET_HEIGHT = 53;
    private final String ALLIANCE_COLOR = "blue";

    // Timing constants (in milliseconds)
    private final double DRIVE_TO_TRIANGLE_TIME = 0.75;      // Time to drive forward to triangle tip
    private final double TURN_TIME = 0.5;                   // Time to turn to face goal
    private final double AUTO_AIM_TIME = 1.5;               // Time to auto-aim
    private final double SHOOT_WAIT_TIME = 1.5;             // Time for ball to be shot and clear
    private final double TILT_DOWN_TIME = 2;              // Time to tilt shooter down
    private final double INTAKE_RAMP_TIME = 0.5;             // Time for intake to ramp up
    private final double LEAVE_ZONE_TIME = 2;             // Time to leave launch zone

    private Shooter.ServoPos shootingAngle = new Shooter.ServoPos(0.53, 0.347);
    private double shootingSpeed = 0.39;

    @Override
    public void runOpMode() {
        this.robot = new Robot(hardwareMap, telemetry, gamepad1, gamepad2);

        waitForStart();

//        // --- STATE 1: Drive forward to the tip of the triangle ---
//        this.robot.drivetrain.driveStraight(0.2);
//        runtime.reset();
//        while (opModeIsActive() && (runtime.seconds() < DRIVE_TO_TRIANGLE_TIME)) {
//            telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
//            telemetry.update();
//        }
//        this.robot.drivetrain.stop();
//        sleep(200);
//
//        // --- STATE 2: Turn to face goal ---
//        this.robot.drivetrain.turnLeft(0.2);
//        runtime.reset();
//        while (opModeIsActive() && (runtime.seconds() < TURN_TIME)) {
//            telemetry.addLine("Turning to face goal...");
//            telemetry.update();
//        }
//        this.robot.drivetrain.stop();
//        sleep(200);

        // --- STATE 3: Auto-aim and position ---

        this.robot.shooter.setTiltAngle(shootingAngle);
        this.robot.shooter.setOutTakePower(shootingSpeed);
//        runtime.reset();
//        while (opModeIsActive() && (runtime.seconds() < 3)) {
//            //this.robot.handleAutoShoot(BASKET_HEIGHT, ALLIANCE_COLOR, false);
//
//            telemetry.addLine("Auto-aiming...");
//            telemetry.update();
//        }
//        this.robot.drivetrain.stop();
//        sleep(200);


        // --- STATE 4: Shoot preload ball (1st ball) ---
        this.robot.shooter.outTake();
        sleep(2250);  // Wait for shooter to spin up before firing
        shootBall();
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < SHOOT_WAIT_TIME)) {
            telemetry.addLine("Shooting ball 1...");
            telemetry.update();
        }

        // --- STATE 5: Shoot second ball ---

        shootBall();
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < SHOOT_WAIT_TIME)) {
            telemetry.addLine("Shooting ball 2...");
            telemetry.update();
        }

        // --- STATE 6: Tilt down to re-adjust last ball ---
        this.robot.tube.outTake();
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < TILT_DOWN_TIME)) {
            telemetry.addLine("Tilting down...");
            telemetry.update();
            this.robot.shooter.setTiltAngle(this.robot.shooter.MIN);
        }
        this.robot.tube.stop();

        // Auto-aim again for third ball
        this.robot.autoShootIsActive = true;
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < AUTO_AIM_TIME)) {
            telemetry.addLine("Auto-aiming...");
            this.robot.shooter.setTiltAngle(shootingAngle);
            this.robot.shooter.setOutTakePower(shootingSpeed);
            telemetry.update();
        }
        this.robot.drivetrain.stop();
        this.robot.autoShootIsActive = false;
        sleep(200);

        // --- STATE 7: Shoot third ball ---

        shootBall();
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < SHOOT_WAIT_TIME)) {
            telemetry.addLine("Shooting ball 3...");
            telemetry.update();
        }

        // --- STATE 8: Leave launch zone ---
        this.robot.drivetrain.driveStraight(0.2);

        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < LEAVE_ZONE_TIME)) {
            telemetry.addLine("Leaving launch zone...");
            telemetry.update();
        }
        this.robot.drivetrain.stop();

        telemetry.addLine("Auto complete!");
        telemetry.update();
    }

    private void shootBall() {
        this.robot.tube.outTake();
        sleep(2000);  // Let tube fire
        this.robot.tube.stop();
    }
}