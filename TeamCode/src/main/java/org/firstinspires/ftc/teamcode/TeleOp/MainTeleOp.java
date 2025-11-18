package org.firstinspires.ftc.teamcode.TeleOp;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.Robot;

@TeleOp(name = "Main TeleOp", group = "Main")
public class MainTeleOp extends LinearOpMode {

    private Robot robot;
    private GamepadEx gp1;

    @Override
    public void runOpMode() {
        gp1 = new GamepadEx(gamepad1);
        robot = new Robot(hardwareMap);
        robot.drivetrain.setPose(new Pose());

        waitForStart();

        while (opModeIsActive()) {

            gp1.readButtons();

            robot.drivetrain.setTeleOpDrive(
                    -gp1.getLeftY(),
                    gp1.getLeftX(),
                    gp1.getRightX(),
                    true
            );

            double triggerPower =
                    gp1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER)
                            - gp1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER);

            robot.shooter.setShooter();
            //robot.loader.setLoaderMotor(triggerPower);

            if (gp1.isDown(GamepadKeys.Button.X)) {
                robot.intake.intakeArtifacts();
            } else if (gp1.isDown(GamepadKeys.Button.B)) {
                robot.intake.intakeArtifacts();
            } else {
                robot.intake.stop();
            }

            robot.run();

            telemetry.addData("Shooter/Loader Power", triggerPower);
            telemetry.update();
        }
    }
}
