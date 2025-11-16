package org.firstinspires.ftc.teamcode.TeleOp;
//
//import com.arcrobotics.ftclib.gamepad.GamepadEx;
//import com.arcrobotics.ftclib.gamepad.GamepadKeys;
//
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//
//import org.firstinspires.ftc.teamcode.Subsystems.Robot;
//
//@TeleOp(name = "Main TeleOp", group = "Main")
//public class MainTeleOp  {

    //private Robot robot;
//    private GamepadEx gp1;
//
//    @Override
//    public void runOpMode() {
//        gp1 = new GamepadEx(gamepad1);
//
//        robot = new Robot(hardwareMap);
//
//        waitForStart();
//
//        while (opModeIsActive()) {
//
//            gp1.readButtons();
//
//            // Mecanum drive
//            robot.drivetrain.setTeleOpDrive(
//                    -gp1.getLeftY(),
//                    gp1.getLeftX(),
//                    gp1.getRightX(),
//                    true
//            );
//
//            // Shooter control
//            double shooterPower =
//                    gp1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER)
//                            - gp1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER);
//            robot.shooter.setShooter();
//
//            // Loader control
//            if (gp1.isDown(GamepadKeys.Button.X)) {
//                robot.loader.setIntake(1);
//            } else if (gp1.isDown(GamepadKeys.Button.B)) {
//                robot.loader.setIntake(-1);
//            } else {
//                robot.loader.stop();
//            }
//
//            robot.run();
//
//            telemetry.addData("Shooter Power", shooterPower);
//            telemetry.update();
//        }
//    }
//}
//
