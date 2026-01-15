package org.firstinspires.ftc.teamcode.decode.TeleOp;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.decode.Subsystems.Robot;

@TeleOp(name = "Better Intake")
@Config
public class Betterintake extends LinearOpMode {

    private Robot robot;
    private GamepadEx gp1;

    // How fast the power ramps per second
    public static double RAMP_RATE = 0.75;

    @Override
    public void runOpMode() {

        robot = new Robot(hardwareMap);
        gp1 = new GamepadEx(gamepad1);

        double intakePower = 0;

        waitForStart();

        while (opModeIsActive()) {

            gp1.readButtons();

            double dt = 0.02; // approximate loop time (20ms)

            // Increase power when holding X
            if (gp1.isDown(GamepadKeys.Button.X)) {
                intakePower += RAMP_RATE * dt;
            }

            // Decrease power when holding B
            if (gp1.isDown(GamepadKeys.Button.B)) { 
                intakePower -= RAMP_RATE * dt;
            }

            // Clamp power to range -1 to 1
            intakePower = Math.max(-1.0, Math.min(1.0, intakePower));

            // Apply power
            robot.intake.intakeArtifacts(intakePower);

            telemetry.addData("Intake Power", intakePower);
            telemetry.update();
        }
    }
}
