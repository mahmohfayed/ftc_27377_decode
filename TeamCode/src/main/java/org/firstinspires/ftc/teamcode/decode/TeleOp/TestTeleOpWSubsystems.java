//package org.firstinspires.ftc.teamcode.decode.TeleOp;
//
//import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.linearOpMode;
//
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//
//import org.firstinspires.ftc.teamcode.Subsystems.BasicRobot;
//
//public class TestTeleOpWSubsystems extends LinearOpMode {
//
//    private BasicRobot robot = new BasicRobot();
//
//    @Override
//    public void runOpMode() {
//        robot.init(hardwareMap);
//
//        waitForStart();
//
//        while (opModeIsActive()) {
//            double y = gamepad1.left_stick_y * 0.5;
//            double x = -gamepad1.left_stick_x;
//            double rx = -gamepad1.right_stick_x * 0.5;
//
//            double rightTrigger = gamepad1.right_trigger;
//            double leftTrigger = gamepad1.left_trigger;
//
//
//            robot.drive();// set the robot so it drives
//
//            robot.intakeArtifacts();
//
//
//            if (gamepad1.y) {// if y is pressed shoot artifacts w loader
//                robot.shooter.shootArtifacts();
//            }
//            else {
//                robot.shooter.stop();
//            }
//
//            if (gamepad1.right_bumper) {// moves arifacts up and into robot
//                robot.loader.loadArtifacts();
//            }
//            else if (gamepad1.left_bumper) {// moves arifacts down and out of robot
//                robot.loader.unloadArtifacts();
//                robot.intake.unloadArtifacts();
//            }
//            else {
//                robot.loader.stop();
//            }
//
//            telemetry.update();
//
//        }
//    }
//
//}
