//package org.firstinspires.ftc.teamcode.Stephon.opmodes.Autonomous;
//
//import com.bylazar.configurables.annotations.Configurable;
//import com.bylazar.telemetry.PanelsTelemetry;
//import com.bylazar.telemetry.TelemetryManager;
//import com.pedropathing.follower.Follower;
//import com.pedropathing.geometry.BezierCurve;
//import com.pedropathing.geometry.BezierLine;
//import com.pedropathing.geometry.Pose;
//import com.pedropathing.paths.Path;
//import com.pedropathing.paths.PathChain;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//
//import org.firstinspires.ftc.teamcode.Stephon.opmodes.Autonomous.pedroPathing.Constants;
//
//
//@Autonomous(name = "Back Blue Auto", group = "Stephon")
//@Configurable // Panels
//public class BackBlueAuto extends OpMode {
//
//    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
//    public Follower follower; // Pedro Pathing follower instance
//    private int pathState; // Current autonomous path state (state machine)
//
//    private final Pose startPose = new Pose(63, 9, Math.toRadians(90));
//    private final Pose scorePose = new Pose(63.000, 15, Math.toRadians(117));
//    private final Pose pickupRow1Pose = new Pose(17, 35.574, Math.toRadians(180));
//    private final Pose pickupRow2Pose = new Pose(17, 59.795, Math.toRadians(180));
//    private final Pose pickupRow3Pose = new Pose(17, 84.016, Math.toRadians(180));
//
//    private Path scorePreload;
//    private PathChain grabPickup1, scorePickup1, grabPickup2, scorePickup2, grabPickup3, scorePickup3;
//
//
//    @Override
//    public void init() {
//        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
//
//        follower = Constants.createFollower(hardwareMap);
//        follower.setStartingPose(new Pose(63, 8.894, Math.toRadians(90)));
//
//        // Build paths
//        firstRowPath = new FirstRowPath(follower);
//
//        panelsTelemetry.debug("Status", "Initialized");
//        panelsTelemetry.update(telemetry);
//    }
//
//    @Override
//    public void loop() {
//        follower.update(); // Update Pedro Pathing
//        pathState = autonomousPathUpdate(); // Update autonomous state machine
//
//        // Log values to Panels and Driver Station
//        panelsTelemetry.debug("Path State", pathState);
//        panelsTelemetry.debug("X", follower.getPose().getX());
//        panelsTelemetry.debug("Y", follower.getPose().getY());
//        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
//        panelsTelemetry.update(telemetry);
//    }
//
//
//    public void buildPaths() {
//        scorePreload = new Path(new BezierLine(startPose, scorePose));
//        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());
//
//        /* --- Row One Scoring --- */
//        grabPickup1 = this.follower.pathBuilder()
//                .addPath(new BezierLine(scorePose, pickupRow1Pose))
//                .setLinearHeadingInterpolation(scorePose.getHeading(), pickupRow1Pose.getHeading())
//                .build();
//
//        scorePickup1 = this.follower.pathBuilder()
//                .addPath(new BezierLine(pickupRow1Pose, scorePose))
//                .setLinearHeadingInterpolation(pickupRow1Pose.getHeading(), scorePose.getHeading())
//                .build();
//
//        /* --- Row Two Scoring --- */
//        grabPickup2 = this.follower.pathBuilder()
//                .addPath(new BezierLine(scorePose, pickupRow2Pose))
//                .setLinearHeadingInterpolation(scorePose.getHeading(), pickupRow2Pose.getHeading())
//                .build();
//
//        scorePickup2 = this.follower.pathBuilder()
//                .addPath(new BezierLine(pickupRow2Pose, scorePose))
//                .setLinearHeadingInterpolation(pickupRow2Pose.getHeading(), scorePose.getHeading())
//                .build();
//
//        /* --- Row Three Scoring --- */
//        grabPickup3 = this.follower.pathBuilder()
//                .addPath(new BezierLine(scorePose, pickupRow3Pose))
//                .setLinearHeadingInterpolation(scorePose.getHeading(), pickupRow3Pose.getHeading())
//                .build();
//
//        scorePickup3 = this.follower.pathBuilder()
//                .addPath(new BezierLine(pickupRow3Pose, scorePose))
//                .setLinearHeadingInterpolation(pickupRow3Pose.getHeading(), scorePose.getHeading())
//                .build();
//
//
//    }
//
//    public void autonomousPathUpdate() {
//        switch (pathState) {
//            case 0: // Go to score preload position
//                follower.followPath(scorePreload);
//                setPathState(1);
//                break;
//            case 1: // Score preload and go to pickup row 1
//                if (!follower.isBusy()) {
//                    /* Score Preload */
//
//                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
//                    follower.followPath(grabPickup1, true);
//                    setPathState(2);
//                }
//                break;
//            case 2: // Grab row 1 and return to scoring position
//                if (!follower.isBusy()) {
//                    /* Grab Sample */
//                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
//                    follower.followPath(scorePickup1,true);
//                    setPathState(3);
//                }
//                break;
//            case 3: // Score preload and go to pickup row 2
//                if(!follower.isBusy()) {
//                    /* Score Sample */
//                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
//                    follower.followPath(grabPickup2,true);
//                    setPathState(4);
//                }
//                break;
//            case 4: // Grab row 2 and return to scoring position
//
//        }
//    }
//
//    public void setPathState(int pState) {
//        pathState = pState;
//        //pathTimer.resetTimer();
//    }
//}
//
//    // Drive forward and rotate to shoot initial balls
//    public static class InitialPath {
//        public PathChain Path1;
//
//
//        public InitialPath(Follower follower) {
//            Path1 = follower.pathBuilder()
//                    .addPath(
//                            new BezierLine(new Pose(63, 9), new Pose(63.000, 15))
//                    )
//                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(117))
//                    .build();
//        }
//
//    }
//
//    public static class FirstRowPath {
//
//        public PathChain Path1;
//        public PathChain Path2;
//
//        public FirstRowPath(Follower follower) {
//
//            Path1 = follower
//                    .pathBuilder()
//                    .addPath(
//                            new BezierCurve(
//                                    new Pose(63.000, 15),
//                                    new Pose(66.607, 37.088),
//                                    new Pose(17.219, 35.574)
//                            )
//                    )
//                    .setLinearHeadingInterpolation(Math.toRadians(117), Math.toRadians(180))
//                    .build();
//
//            Path2 = follower
//                    .pathBuilder()
//                    .addPath(
//                            new BezierCurve(
//                                    new Pose(17.219, 35.574),
//                                    new Pose(66.607, 37.088),
//                                    new Pose(63.000, 15)
//                            )
//                    )
//                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(117))
//                    .build();
//        }
//    }
//
//    public static class SecondRowPath {
//
//        public PathChain Path1;
//        public PathChain Path2;
//
//        public SecondRowPath(Follower follower) {
//
//            Path1 = follower
//                    .pathBuilder()
//                    .addPath(
//                            new BezierCurve(
//                                    new Pose(63.000, 15),
//                                    new Pose(68.689, 64.526),
//                                    new Pose(18.355, 59.795)
//                            )
//                    )
//                    .setLinearHeadingInterpolation(Math.toRadians(117), Math.toRadians(180))
//                    .build();
//
//            Path2 = follower
//                    .pathBuilder()
//                    .addPath(
//                            new BezierCurve(
//                                    new Pose(18.355, 59.795),
//                                    new Pose(68.689, 64.526),
//                                    new Pose(63.000, 15)
//                            )
//                    )
//                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(117))
//                    .build();
//        }
//    }
//
//    public static class ThirdRowPath {
//
//        public PathChain Path1;
//        public PathChain Path2;
//
//        public ThirdRowPath(Follower follower) {
//
//            Path1 = follower
//                    .pathBuilder()
//                    .addPath(
//                            new BezierCurve(
//                                    new Pose(63.000, 15),
//                                    new Pose(55.254, 91.206),
//                                    new Pose(78.907, 81.556),
//                                    new Pose(18.166, 84.016)
//                            )
//                    )
//                    .setLinearHeadingInterpolation(Math.toRadians(117), Math.toRadians(180))
//                    .build();
//
//            Path2 = follower
//                    .pathBuilder()
//                    .addPath(
//                            new BezierCurve(
//                                    new Pose(18.166, 84.016),
//                                    new Pose(78.907, 81.556),
//                                    new Pose(55.254, 91.206),
//                                    new Pose(63.000, 15)
//                            )
//                    )
//                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(117))
//                    .build();
//        }
//
//
//
//
//}