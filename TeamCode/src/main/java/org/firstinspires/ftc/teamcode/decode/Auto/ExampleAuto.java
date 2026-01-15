//package org.firstinspires.ftc.teamcode.decode.Auto;
//
//import static org.firstinspires.ftc.teamcode.decode.Subsystems.Common.robot;
//
//import com.acmerobotics.roadrunner.ParallelAction;
//import com.acmerobotics.roadrunner.SequentialAction;
//import com.pedropathing.follower.Follower;
//import com.pedropathing.geometry.Pose;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//
//import org.firstinspires.ftc.teamcode.decode.Subsystems.Actions;
//import org.firstinspires.ftc.teamcode.decode.Subsystems.Common;
//import org.firstinspires.ftc.teamcode.decode.Subsystems.FollowPathAction;
//import org.firstinspires.ftc.teamcode.decode.Subsystems.RobotActions;
//
//@Autonomous(name = "EXAMPLE_AUTO")
//public class ExampleAuto extends AbstractAuto {
//
//    private Follower follower;
//    private AudiencePath paths;
//
//    // -----------------------------
//    // START POSE
//    // -----------------------------
//    @Override
//    protected Pose getStartPose() {
//        return AudiencePath.P_START; // your pose variable
//    }
//
//    // -----------------------------
//    // INIT
//    // -----------------------------
//    @Override
//    protected void onInit() {
//
//        follower = robot.drivetrain;
//        paths = new AudiencePath(follower);
//
//        // ---- alliance mirroring ----
//        if (Common.isRed != AudiencePath.isPathRed) {
//            AudiencePath.isPathRed = !AudiencePath.isPathRed;
//            paths.mirrorAll();
//        }
//
//        // ---- build the paths you want ----
//        paths.goal6Build(); // choose goal3 / goal6 / goal9 / etc
//    }
//
//    // -----------------------------
//    // RUN
//    // -----------------------------
//    @Override
//    protected void onRun() {
//
//        runPreload();
//        runIntakeAndShoot();
//        runLeave();
//    }
//
//    // -----------------------------
//    // PRELOAD EXAMPLE
//    // -----------------------------
//    private void runPreload() {
//
//        robot.actionScheduler.addAction(
//                new SequentialAction(
//
//                        // PATH + ACTION AT SAME TIME
//                        new ParallelAction(
//                                new Actions.CallbackAction(
//                                        RobotActions.startShooter(0.35),
//                                        paths.shootpreload,
//                                        0.2,          // t trigger
//                                        0,            // path index
//                                        follower,
//                                        "Rev Shooter"
//                                ),
//                                new FollowPathAction(follower, paths.shootpreload, true)
//                        ),
//
//                        // ACTION AFTER PATH
//                        new ParallelAction(
//                                RobotActions.loaderAction(1, 1.5)
//                        )
//                )
//        );
//
//        robot.actionScheduler.runBlocking();
//    }
//
//    // -----------------------------
//    // INTAKE + SHOOT EXAMPLE
//    // -----------------------------
//    private void runIntakeAndShoot() {
//
//        robot.actionScheduler.addAction(
//                new SequentialAction(
//
//                        // Intake while driving
//                        new ParallelAction(
//                                new Actions.CallbackAction(
//                                        RobotActions.intakeAction(1, 3),
//                                        paths.intake,
//                                        0.3,
//                                        0,
//                                        follower,
//                                        "Intake Balls"
//                                ),
//                                new FollowPathAction(follower, paths.intake)
//                        ),
//
//                        // Shoot after intake
//                        new ParallelAction(
//                                RobotActions.startShooter(0.35),
//                                RobotActions.loaderAction(1, 2)
//                        ),
//
//                        // Shoot path
//                        new FollowPathAction(follower, paths.shoot3)
//                )
//        );
//
//        robot.actionScheduler.runBlocking();
//    }
//
//    // -----------------------------
//    // LEAVE EXAMPLE
//    // -----------------------------
//    private void runLeave() {
//
//        robot.actionScheduler.addAction(
//                new FollowPathAction(follower, paths.leave)
//        );
//
//        robot.actionScheduler.runBlocking();
//    }
//}
