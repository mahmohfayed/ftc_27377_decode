//package org.firstinspires.ftc.teamcode.decode.TeleOp;
//
//
//// In progress dont change
//
//import com.arcrobotics.ftclib.gamepad.GamepadEx;
//import com.arcrobotics.ftclib.gamepad.GamepadKeys;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//
//import org.firstinspires.ftc.teamcode.Subsystems.BasicRobot;
//
//public class TeleOpWPS5 extends LinearOpMode {
//
//    private BasicRobot robot = new BasicRobot();
//    private GamepadEx gp1;
//
//    @Override
//    public void runOpMode() {
//        gp1 = new GamepadEx(gamepad1);
//        robot.init(hardwareMap);
//
//        waitForStart();
//
//        while (opModeIsActive()) {
//            gp1.readButtons();
//
//
//            double triggerPower =
//                    gp1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER)
//                            - gp1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER);
//
//            robot.shooter.shootArtifacts();
//
//            if (gp1.isDown(GamepadKeys.Button.X)) { // down controller
//                robot.intake.intakeArtifacts(1);
//            } else if (gp1.isDown(GamepadKeys.Button.B)) {
//                robot.intake.intakeArtifacts(-1);
//            } else {
//                robot.intake.stop();
//            }
//
//
//            robot.drive();// set the robot so it drives
//
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
