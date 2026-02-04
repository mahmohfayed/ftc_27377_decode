package org.firstinspires.ftc.teamcode.decode.testing;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@Disabled
@TeleOp(name = "MotorTest (PS5)")
public class MotorTest extends LinearOpMode {

    private DcMotor testMotor;
    private GamepadEx gamepadEx1;

    @Override
    public void runOpMode() {
        // Change "testMotor" to whatever your motor is named in the Config
        testMotor = hardwareMap.get(DcMotor.class, "testMotor");
        gamepadEx1 = new GamepadEx(gamepad1);

        telemetry.addLine("Ready! Use PS5 Cross (X) or Right Trigger");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            gamepadEx1.readButtons();

            // Use the Cross button (mapped as A) for full power
            if (gamepadEx1.getButton(GamepadKeys.Button.A)) {
                testMotor.setPower(1.0);
            }
            // Or use the Right Trigger for variable power
            else if (gamepadEx1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.1) {
                testMotor.setPower(gamepadEx1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER));
            }
            else {
                testMotor.setPower(0);
            }

            telemetry.addData("Motor Power", testMotor.getPower());
            telemetry.update();
        }
    }
}