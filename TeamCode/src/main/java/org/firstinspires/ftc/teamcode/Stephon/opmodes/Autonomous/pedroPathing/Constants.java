package org.firstinspires.ftc.teamcode.Stephon.opmodes.Autonomous.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(11.339809)

            // Automatic Tuners
            .forwardZeroPowerAcceleration(-31.21165195)
            .lateralZeroPowerAcceleration(-73.5773664);

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("rightfront")
            .rightRearMotorName("rightback")
            .leftRearMotorName("leftback")
            .leftFrontMotorName("leftfront")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)

            // Automatic Tuners
            .xVelocity(76.20739)
            .yVelocity(50.69899);


    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(-8)  // Keep this for now, we'll verify later
            .strafePodX(0)     // Keep this for now, we'll verify later
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)

            // THESE ARE THE KEY CHANGES:
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)  // Changed from REVERSED
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED); // Changed from FORWARD

    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .pinpointLocalizer(localizerConstants)
                .build();
    }
}
