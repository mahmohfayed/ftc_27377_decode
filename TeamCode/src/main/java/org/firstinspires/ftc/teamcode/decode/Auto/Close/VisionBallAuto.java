//package org.firstinspires.ftc.teamcode.decode.Auto.Close;
//
//import static org.firstinspires.ftc.teamcode.decode.Subsystems.Common.robot;
//
//import com.acmerobotics.roadrunner.InstantAction;
//import com.acmerobotics.roadrunner.ParallelAction;
//import com.acmerobotics.roadrunner.SequentialAction;
//import com.acmerobotics.roadrunner.SleepAction;
//import com.bylazar.configurables.annotations.Configurable;
//import com.pedropathing.follower.Follower;
//import com.pedropathing.geometry.BezierLine;
//import com.pedropathing.geometry.BezierCurve;
//import com.pedropathing.geometry.Pose;
//import com.pedropathing.paths.PathChain;
//import com.qualcomm.hardware.limelightvision.LLResult;
//import com.qualcomm.hardware.limelightvision.LLResultTypes;
//import com.qualcomm.hardware.limelightvision.Limelight3A;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//
//import org.firstinspires.ftc.teamcode.decode.Auto.AbstractAuto;
//import org.firstinspires.ftc.teamcode.decode.Subsystems.Actions;
//import org.firstinspires.ftc.teamcode.decode.Subsystems.Common;
//import org.firstinspires.ftc.teamcode.decode.Subsystems.FollowPathAction;
//import org.firstinspires.ftc.teamcode.decode.Subsystems.LimelightEx;
//import org.firstinspires.ftc.teamcode.decode.Subsystems.RobotActions;
//
//import java.util.List;
//
//@Configurable
//@Autonomous(name = "VisionBallAuto", group = "Main")
//public class VisionBallAuto extends AbstractAuto {
//
//    private Follower f;
//    private Paths path;
//    private LimelightEx limelight;
//
//    // ── Pipelines ──────────────────────────────────────────────────────────
//    public static int BALL_PIPELINE     = 1;
//    public static int APRILTAG_PIPELINE = 0;
//
//    // ── Timing ────────────────────────────────────────────────────────────
//    public static double SPIKE_INTAKE_TIME   = 1.5;
//    public static double CLUSTER_INTAKE_TIME = 3.0;
//    public static double SHOOTER_SPINUP      = 1.0;
//    public static double SHOOT_TIME          = 2.0;
//    public static double SHOOT_HOOD          = 0.4;
//
//    // ── Vision scan ───────────────────────────────────────────────────────
//    public static double MIN_BALL_CONFIDENCE = 0.50;
//    public static int    MAX_SCAN_ATTEMPTS   = 5;
//    public static double SCAN_STEP           = 8.0;
//
//    // ── Camera mount — tune to your robot ─────────────────────────────────
//    public static double CAM_HEIGHT_IN = 9.0;
//    public static double CAM_MOUNT_DEG = 30.0;
//
//    // ══════════════════════════════════════════════════════════════════════
//    // AbstractAuto overrides
//    // ══════════════════════════════════════════════════════════════════════
//
//    @Override
//    protected Pose getStartPose() {
//        return Paths.P_START;
//    }
//
//    @Override
//    protected void onInit() {
//        f    = robot.drivetrain;
//        path = new Paths(f);
//
//        limelight = new LimelightEx(
//                hardwareMap.get(Limelight3A.class, "limelight")
//        );
//
//        if (Common.isRed != Paths.isPathRed) {
//            Paths.isPathRed = !Paths.isPathRed;
//            path.mirrorAll();
//        }
//
//        path.goal21Build();
//    }
//
//    @Override
//    protected void onRun() {
//        shootPreload();
//        collectSpike3();
//        returnAndShoot(path.shoot3);
//        collectSpike6();
//        returnAndShoot(path.shoot6);
//        driveToScanPosition();
//        Pose clusterPose = scanForCluster();
//        if (clusterPose != null) {
//            collectCluster(clusterPose);
//            returnAndShootFromCluster();
//        } else {
//            telemetry.addLine("No cluster found — ending auto");
//            telemetry.update();
//        }
//    }
//
//    // ══════════════════════════════════════════════════════════════════════
//    // Preload
//    // ══════════════════════════════════════════════════════════════════════
//
//    private void shootPreload() {
//        robot.actionScheduler.addAction(
//                new SequentialAction(
//                        new ParallelAction(
//                                new Actions.CallbackAction(
//                                        RobotActions.startShooter(1),
//                                        path.shootPreload, 0.3, 0, f, "PreloadRev"
//                                ),
//                                new FollowPathAction(f, path.shootPreload, true)
//                        ),
//                        new ParallelAction(
//                                RobotActions.intakeAction(1, 1.5),
//                                RobotActions.loaderAction(1, 1.5),
//                                RobotActions.setHoodServo(SHOOT_HOOD)
//                        ),
//                        new InstantAction(() -> robot.shooter.stop())
//                )
//        );
//        robot.actionScheduler.runBlocking();
//    }
//
//    // ══════════════════════════════════════════════════════════════════════
//    // Spike mark collection
//    // ══════════════════════════════════════════════════════════════════════
//
//    private void collectSpike3() {
//        path.intake3.getPath(0).setBrakingStart(0.9);
//        path.intake3.getPath(0).setBrakingStrength(0.7);
//
//        robot.actionScheduler.addAction(
//                new SequentialAction(
//                        new ParallelAction(
//                                new InstantAction(() -> f.setMaxPower(0.7)),
//                                new Actions.CallbackAction(
//                                        RobotActions.intakeAction(1, SPIKE_INTAKE_TIME + 2),
//                                        path.intake3, 0.01, 0, f, "IntakeSpike3"
//                                ),
//                                new FollowPathAction(f, path.intake3)
//                        ),
//                        new ParallelAction(
//                                RobotActions.intakeAction(1, SPIKE_INTAKE_TIME),
//                                new SleepAction(SPIKE_INTAKE_TIME)
//                        ),
//                        new InstantAction(() -> f.setMaxPower(1))
//                )
//        );
//        robot.actionScheduler.runBlocking();
//    }
//
//    private void collectSpike6() {
//        path.intake6.getPath(0).setBrakingStart(0.8);
//        path.intake6.getPath(0).setBrakingStrength(0.8);
//
//        robot.actionScheduler.addAction(
//                new SequentialAction(
//                        new ParallelAction(
//                                new InstantAction(() -> f.setMaxPower(0.7)),
//                                new Actions.CallbackAction(
//                                        RobotActions.intakeAction(1, SPIKE_INTAKE_TIME + 2),
//                                        path.intake6, 0.01, 0, f, "IntakeSpike6"
//                                ),
//                                new FollowPathAction(f, path.intake6)
//                        ),
//                        new ParallelAction(
//                                RobotActions.intakeAction(1, SPIKE_INTAKE_TIME),
//                                new SleepAction(SPIKE_INTAKE_TIME)
//                        ),
//                        new InstantAction(() -> f.setMaxPower(1))
//                )
//        );
//        robot.actionScheduler.runBlocking();
//    }
//
//    // ══════════════════════════════════════════════════════════════════════
//    // Return and shoot
//    // ══════════════════════════════════════════════════════════════════════
//
//    private void returnAndShoot(PathChain shootPath) {
//        shootPath.getPath(0).setBrakingStart(0.8);
//        shootPath.getPath(0).setBrakingStrength(0.7);
//
//        robot.actionScheduler.addAction(
//                new SequentialAction(
//                        new ParallelAction(
//                                new InstantAction(() -> f.setMaxPower(1)),
//                                new Actions.CallbackAction(
//                                        RobotActions.startShooter(SHOOTER_SPINUP),
//                                        shootPath, 0.4, 0, f, "SpinUp"
//                                ),
//                                new FollowPathAction(f, shootPath, true)
//                        ),
//                        new ParallelAction(
//                                RobotActions.setHoodServo(SHOOT_HOOD),
//                                RobotActions.intakeAction(1, SHOOT_TIME),
//                                RobotActions.loaderAction(1, SHOOT_TIME)
//                        ),
//                        new InstantAction(() -> robot.shooter.stop())
//                )
//        );
//        robot.actionScheduler.runBlocking();
//    }
//
//    // ══════════════════════════════════════════════════════════════════════
//    // Scan staging
//    // ══════════════════════════════════════════════════════════════════════
//
//    private void driveToScanPosition() {
//        robot.actionScheduler.addAction(
//                new FollowPathAction(f, path.unloadRamp, true)
//        );
//        robot.actionScheduler.runBlocking();
//        try { Thread.sleep(300); } catch (InterruptedException e) { Thread.currentThread().interrupt(); }
//    }
//
//    // ══════════════════════════════════════════════════════════════════════
//    // Vision scan
//    // ══════════════════════════════════════════════════════════════════════
//
//    private Pose scanForCluster() {
//        limelight.getLimelight().pipelineSwitch(BALL_PIPELINE);
//        try { Thread.sleep(200); } catch (InterruptedException e) { Thread.currentThread().interrupt(); }
//
//        for (int attempt = 0; attempt < MAX_SCAN_ATTEMPTS; attempt++) {
//            limelight.update();
//            LLResult result = limelight.getResult();
//
//            if (result != null && result.isValid()) {
//                LLResultTypes.DetectorResult best = getBestDetection(result);
//
//                if (best != null) {
//                    Pose clusterPose = estimateClusterPose(best);
//                    telemetry.addData("Cluster found — attempt", attempt + 1);
//                    telemetry.addData("Cluster X", "%.1f", clusterPose.getX());
//                    telemetry.addData("Cluster Y", "%.1f", clusterPose.getY());
//                    telemetry.update();
//                    limelight.getLimelight().pipelineSwitch(APRILTAG_PIPELINE);
//                    return clusterPose;
//                }
//            }
//
//            if (attempt < MAX_SCAN_ATTEMPTS - 1) {
//                sweepStep();
//            }
//
//            telemetry.addData("Scan attempt", attempt + 1);
//            telemetry.update();
//        }
//
//        telemetry.addLine("No cluster found");
//        telemetry.update();
//        limelight.getLimelight().pipelineSwitch(APRILTAG_PIPELINE);
//        return null;
//    }
//
//    private LLResultTypes.DetectorResult getBestDetection(LLResult result) {
//        List<LLResultTypes.DetectorResult> detections = result.getDetectorResults();
//        if (detections == null || detections.isEmpty()) return null;
//
//        LLResultTypes.DetectorResult best = null;
//        double bestConf = MIN_BALL_CONFIDENCE;
//
//        for (LLResultTypes.DetectorResult det : detections) {
//            if (det.getConfidence() > bestConf) {
//                bestConf = det.getConfidence();
//                best     = det;
//            }
//        }
//        return best;
//    }
//
//    private Pose estimateClusterPose(LLResultTypes.DetectorResult det) {
//        double tx = det.getTargetXDegrees();
//        double ty = det.getTargetYDegrees();
//
//        double angleToTarget = Math.toRadians(CAM_MOUNT_DEG + ty);
//        double groundDist    = (Math.abs(angleToTarget) > 0.01)
//                ? CAM_HEIGHT_IN / Math.tan(angleToTarget)
//                : 60.0;
//
//        double lateralOffset = groundDist * Math.tan(Math.toRadians(tx));
//
//        Pose   robotPose = robot.drivetrain.getPose();
//        double heading   = robotPose.getHeading();
//
//        double fieldX = robotPose.getX()
//                + groundDist    * Math.cos(heading)
//                - lateralOffset * Math.sin(heading);
//        double fieldY = robotPose.getY()
//                + groundDist    * Math.sin(heading)
//                + lateralOffset * Math.cos(heading);
//
//        double headingToCluster = Math.atan2(
//                fieldY - robotPose.getY(),
//                fieldX - robotPose.getX());
//
//        return new Pose(fieldX, fieldY, headingToCluster);
//    }
//
//    private void sweepStep() {
//        Pose   current = robot.drivetrain.getPose();
//        double heading = current.getHeading();
//
//        Pose target = new Pose(
//                current.getX() + SCAN_STEP * Math.cos(heading),
//                current.getY() + SCAN_STEP * Math.sin(heading),
//                heading
//        );
//
//        PathChain step = f.pathBuilder()
//                .addPath(new BezierLine(current, target))
//                .setConstantHeadingInterpolation(heading)
//                .build();
//
//        robot.actionScheduler.addAction(new FollowPathAction(f, step, true));
//        robot.actionScheduler.runBlocking();
//        try { Thread.sleep(150); } catch (InterruptedException e) { Thread.currentThread().interrupt(); }
//    }
//
//    // ══════════════════════════════════════════════════════════════════════
//    // Cluster collection
//    // ══════════════════════════════════════════════════════════════════════
//
//    private void collectCluster(Pose clusterPose) {
//        Pose current = robot.drivetrain.getPose();
//
//        PathChain toCluster = f.pathBuilder()
//                .addPath(new BezierLine(current, clusterPose))
//                .setLinearHeadingInterpolation(current.getHeading(), clusterPose.getHeading())
//                .build();
//
//        robot.actionScheduler.addAction(
//                new SequentialAction(
//                        new ParallelAction(
//                                new Actions.CallbackAction(
//                                        RobotActions.intakeAction(1, CLUSTER_INTAKE_TIME + 1),
//                                        toCluster, 0.05, 0, f, "IntakeCluster"
//                                ),
//                                new FollowPathAction(f, toCluster, true)
//                        ),
//                        new ParallelAction(
//                                RobotActions.intakeAction(1, 0.75),
//                                new SleepAction(0.75)
//                        )
//                )
//        );
//        robot.actionScheduler.runBlocking();
//    }
//
//    private void returnAndShootFromCluster() {
//        Pose current = robot.drivetrain.getPose();
//
//        PathChain toShoot = f.pathBuilder()
//                .addPath(new BezierLine(current, Paths.P_SHOOT))
//                .setLinearHeadingInterpolation(current.getHeading(), Paths.H_38)
//                .build();
//
//        toShoot.getPath(0).setBrakingStart(0.8);
//        toShoot.getPath(0).setBrakingStrength(0.7);
//
//        robot.actionScheduler.addAction(
//                new SequentialAction(
//                        new ParallelAction(
//                                new InstantAction(() -> f.setMaxPower(1)),
//                                new Actions.CallbackAction(
//                                        RobotActions.startShooter(SHOOTER_SPINUP),
//                                        toShoot, 0.4, 0, f, "SpinUpCluster"
//                                ),
//                                new FollowPathAction(f, toShoot, true)
//                        ),
//                        new ParallelAction(
//                                RobotActions.setHoodServo(SHOOT_HOOD),
//                                RobotActions.intakeAction(1, SHOOT_TIME),
//                                RobotActions.loaderAction(1, SHOOT_TIME)
//                        ),
//                        new InstantAction(() -> robot.shooter.stop())
//                )
//        );
//        robot.actionScheduler.runBlocking();
//    }
//}