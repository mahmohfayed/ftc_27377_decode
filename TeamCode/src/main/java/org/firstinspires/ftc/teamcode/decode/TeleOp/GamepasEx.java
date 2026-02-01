package org.firstinspires.ftc.teamcode.decode.TeleOp;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

// Importing your specific subsystem
import org.firstinspires.ftc.teamcode.decode.Subsystems.HoodServo;

@TeleOp(name = " GAmepadex", group = "TeleOp")
public class GamepasEx extends LinearOpMode {

    // Instantiate your HoodServo subsystem
    //HoodServo hood = new HoodServo();
    GamepadEx gamepadEx1;

    @Override
    public void runOpMode() throws InterruptedException {
        // Use your subsystem's init method
        //ood.init(hardwareMap);
        DcMotor intake = hardwareMap.dcMotor.get("intakeMotor");

        // Initialize GamepadEx for PS5 support
        gamepadEx1 = new GamepadEx(gamepad1);

        telemetry.addLine("Ready - Using HoodServo Subsystem");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Must call readButtons() for GamepadEx to work
            gamepadEx1.readButtons();

            // Mapping PS5 buttons to your subsystem methods
            if (gamepadEx1.getButton(GamepadKeys.Button.A)) { // PS5 Cross
            intake.setPower(1);
            }

            else if (gamepadEx1.getButton(GamepadKeys.Button.Y)) { // PS5 Triangle

            intake.setPower(-1);
            }
            else {
                intake.setPower(0);
            }

            // Using your subsystem's getPosition() for telemetry
           // telemetry.addData("Servo Position", hood.getPosition());
            telemetry.update();
        }
    }
}