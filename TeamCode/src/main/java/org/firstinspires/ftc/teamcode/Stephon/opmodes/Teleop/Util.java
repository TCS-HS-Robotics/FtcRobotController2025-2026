package org.firstinspires.ftc.teamcode.Stephon.opmodes.Teleop;

import com.pedropathing.telemetry.SelectableOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Stephon.Robot;

@TeleOp(name = "Utilities", group = "Stephon")
public class Util extends SelectableOpMode {

    public Util() {
        super("Select Alliance", s -> {
            s.add("Shooter Calibration", ShooterCalibration::new);
        });
    }

}


class ShooterCalibration extends OpMode {
    private Robot robot;

    @Override
    public void init() {
        robot = new Robot(hardwareMap, telemetry, gamepad1, gamepad2);
        telemetry.addLine("This is a Configuration Teleop.");
        telemetry.addLine("Dpad: Changes motor speed \nLeft Joystick: Changes tilt angle");
        telemetry.addLine("Use printed values to calibrate.");
    }

    @Override
    public void loop() {
        robot.shooter.angleControl(true);

        double outTakePower = robot.shooter.getOutTakePower();
        if (gamepad2.dpad_up) {
            robot.shooter.setOutTakePower(outTakePower + 0.01);
        } else if (gamepad2.dpad_down) {
            robot.shooter.setOutTakePower(outTakePower - 0.01);
        }
        robot.handleIntakeOuttake(true);
        telemetry.update();
    }

}