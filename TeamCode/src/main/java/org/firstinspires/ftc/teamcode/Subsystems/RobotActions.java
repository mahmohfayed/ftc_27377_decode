////package org.firstinspires.ftc.teamcode.Subsystems;
////
////import com.acmerobotics.roadrunner.Action;
////import com.acmerobotics.roadrunner.InstantAction;
////import com.acmerobotics.roadrunner.SequentialAction;
////import com.acmerobotics.roadrunner.ParallelAction;
////import com.acmerobotics.roadrunner.SleepAction;
////
////public class RobotActions {
////
////    private static Robot robot;
////
////    public RobotActions(Robot robotInstance) {
////        robot = robotInstance;
////    }
////
////    public static Action shooterAction(double power, double time) {
////        return new ParallelAction(
////                new InstantAction(() -> robot.shooter.setShooter()),
//                new SleepAction(0)
////        );
////    }
////
////    public static Action loaderAction(double power, double time) {
////        return new ParallelAction(
////                new InstantAction(() -> robot.loader.setLoaderMotor(power)),
////                new SleepAction(0)
////        );
////    }
////
////    public static Action intakeAction(double power, double time) {
////        return new ParallelAction(
////                new InstantAction(() -> robot.intake.intakeArtifacts(power)),
////                new SleepAction(0)
////        );
////    }
////
////    public static Action stopAll() {
////        return new InstantAction(() -> {
////            robot.shooter.stop();
////            robot.loader.stop();
////            robot.intake.stop();
////        });
////    }
////
////    public static Action fullSequence() {
////        return new SequentialAction(
////                shooterAction(1.0, 0.5),
////                loaderAction(1.0, 0.5),
////                intakeAction(1.0, 0.5),
////                stopAll()
////        );
////    }
////}
