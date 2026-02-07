package org.firstinspires.ftc.teamcode.decode.TeleOp;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

// Importing your specific subsystem
import org.firstinspires.ftc.teamcode.decode.Subsystems.HoodServo;

@TeleOp(name = "hoodservo test", group = "TeleOp")
public class hoodservotest extends LinearOpMode {

    // Instantiate your HoodServo subsystem
    //HoodServo hood = new HoodServo();
    //GamepadEx gamepadEx1;
    Servo hood;


    @Override
    public void runOpMode() throws InterruptedException {
        // Use your subsystem's init method
        //hood.init(hardwareMap);
        hood = hardwareMap.get(Servo.class, "leftServo");

        // Initialize GamepadEx for PS5 support

        hood.setPosition(0.4);




        waitForStart();

        while (opModeIsActive()) {
            // Must call readButtons() for GamepadEx to work
            if (gamepad1.x){
                hood.setPosition(0.3);
            }
            else if (gamepad1.b){
                hood.setPosition(0.5);
            }
            // Using your subsystem's getPosition() for telemetry
            telemetry.addData("Servo Position", hood.getPosition());
            telemetry.update();
        }
    }
}