package org.firstinspires.ftc.teamcode.Stephon.opmodes.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Stephon.Robot;
import org.firstinspires.ftc.teamcode.Stephon.hardware.Shooter;

@Autonomous(name = "FrontAuto", group = "Stephon")
public class FrontStephonAuto extends LinearOpMode {

    private Robot robot;
    private ElapsedTime runtime = new ElapsedTime();

    private Shooter.ServoPos shootingAngle = new Shooter.ServoPos(0.608, 0.398);
    private double shootingSpeed = 0.3;

    // CONSTANTS
    private final double MOVE_BACK_TIME = 3;
    private final double SHOOT_WAIT_TIME = 1.5;
    private final double TILT_DOWN_TIME = 1.85;
    private final double STRAFE_TIME = 2.5;



    @Override
    public void runOpMode() {
        this.robot = new Robot(hardwareMap, telemetry, gamepad1, gamepad2);

        waitForStart();

        this.robot.shooter.setTiltAngle(shootingAngle);
        this.robot.shooter.setOutTakePower(shootingSpeed);

        // Move back to mid triangle
        this.robot.shooter.outTake();
        this.robot.drivetrain.driveBackward(0.2);

        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < MOVE_BACK_TIME)) {
            telemetry.addLine("Moving backwards...");
            telemetry.update();
        }
        this.robot.drivetrain.stop();

        // Shoot ball 1
        shootBall();
        runtime.reset();
        while(opModeIsActive() && (runtime.seconds() < SHOOT_WAIT_TIME)) {
            telemetry.addLine("Shooting Ball 1...");
            telemetry.update();
        }

        // Shoot ball 2
        shootBall();
        runtime.reset();
        while(opModeIsActive() && (runtime.seconds() < SHOOT_WAIT_TIME)) {
            telemetry.addLine("Shooting Ball 2...");
            telemetry.update();
        }

        // --- Tilt down to re-adjust last ball ---
        this.robot.tube.outTake();
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < TILT_DOWN_TIME)) {
            telemetry.addLine("Tilting down...");
            telemetry.update();
            this.robot.shooter.setTiltAngle(this.robot.shooter.MIN);
        }
        this.robot.tube.stop();

        // Shoot third ball
        this.robot.shooter.setTiltAngle(shootingAngle);
        this.robot.shooter.setOutTakePower(shootingSpeed);
        while (opModeIsActive() && (runtime.seconds() < 1.5)) {
            telemetry.addLine("Tilting up...");
            telemetry.update();
        }

        shootBall();
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < SHOOT_WAIT_TIME)) {
            telemetry.addLine("Shooting ball 3...");
            telemetry.update();
        }

        // Strafe right to leave launch zone
        this.robot.drivetrain._strafeRight(0.2);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < STRAFE_TIME)) {

        }

        this.robot.drivetrain.stop();

    }

    private void shootBall() {
        this.robot.tube.outTake();
        sleep(2000);  // Let tube fire
        this.robot.tube.stop();
    }

}
