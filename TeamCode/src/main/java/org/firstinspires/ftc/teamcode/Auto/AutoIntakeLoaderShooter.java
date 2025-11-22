//package org.firstinspires.ftc.teamcode.Auto;
//
//import com.acmerobotics.roadrunner.Action;
//import com.acmerobotics.roadrunner.Pose2d;
//import com.acmerobotics.roadrunner.SequentialAction;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//
//import org.firstinspires.ftc.teamcode.Subsystems.Robot;
//import org.firstinspires.ftc.teamcode.Subsystems.RobotActions;
//
//@Autonomous(name = "Intake Loader Shooter Auto")
//public class AutoIntakeLoaderShooter extends com.qualcomm.robotcore.eventloop.opmode.LinearOpMode {
//
//    private Robot robot;
//    private RobotActions actions;
//
//    @Override
//    public void runOpMode() {
//        robot = new Robot(hardwareMap);
//        actions = new RobotActions(robot);
//
//        waitForStart();
//
//        if (isStopRequested()) return;
//
//        Action sequence = new SequentialAction(
//                RobotActions.intakeAction(1.0, 0.5),
//                RobotActions.loaderAction(1.0, 0.5),
//                RobotActions.shooterAction(1.0, 0.5),
//                RobotActions.stopAll()
//        );
//
//        sequence.run(robot);
//    }
//}
