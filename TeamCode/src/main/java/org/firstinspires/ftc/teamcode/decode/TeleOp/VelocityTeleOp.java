package org.firstinspires.ftc.teamcode.decode.TeleOp;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.decode.Subsystems.Robot;

@TeleOp(name = "VelocityTeleOp", group = "Main")
public class VelocityTeleOp extends LinearOpMode {

    private Robot robot;
    private GamepadEx gp1;

    @Override
    public void runOpMode() {
        gp1 = new GamepadEx(gamepad1);
        robot = new Robot(hardwareMap);
        robot.drivetrain.setPose(new Pose(0,0,0));

        waitForStart();

        while (opModeIsActive()) {

            gp1.readButtons();

            // Drivetrain control
            robot.drivetrain.setTeleOpDrive(
                    -gp1.getLeftY(),
                    gp1.getLeftX(),
                    gp1.getRightX(),
                    true
            );

            double triggerPower =
                    gp1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER)
                            - gp1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER);

            // Shooter velocity control - use right trigger to control speed
            if (gp1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.1) {
                // Spin up to 2600 ticks/sec when right trigger is pressed
                robot.shooter.setVelocity(2200);
            } else if (gp1.isDown(GamepadKeys.Button.A)) {
                // Alternative: Press A to spin up shooter
                robot.shooter.setVelocity(1750);
            } else if (gp1.isDown(GamepadKeys.Button.Y)) {
                // Press Y to stop shooter
                robot.shooter.stop();
            } else {
                // Stop if no input
                robot.shooter.stop();
            }

            //robot.loader.setLoaderMotor(triggerPower);

            // Intake control
            if (gp1.isDown(GamepadKeys.Button.X)) {
                robot.intake.intakeArtifacts(0.90);
            } else if (gp1.isDown(GamepadKeys.Button.B)) {
                robot.intake.intakeArtifacts(-0.50);
            } else {
                robot.intake.stop();
            }

            robot.run();

            // Telemetry
            telemetry.addData("Shooter/Loader Power", triggerPower);
            robot.shooter.displayTelemetry(telemetry);
            telemetry.addData("Target Velocity", 2600);
            telemetry.update();
        }
    }
}