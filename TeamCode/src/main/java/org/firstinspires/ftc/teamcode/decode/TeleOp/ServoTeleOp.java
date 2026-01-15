package org.firstinspires.ftc.teamcode.decode.TeleOp;

//import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
//import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.decode.Subsystems.HoodServo;

@TeleOp
public class ServoTeleOp extends OpMode{

        //HoodServo servo = new HoodServo();
        public Servo servo;
        @Override
        public void init(){
            //servo.init(hardwareMap);
            servo = hardwareMap.get(Servo.class, "hoodServo");

        }
        @Override
        public void loop(){
            servo.setDirection(Servo.Direction.REVERSE);
            if (gamepad1.a){
                servo.setPosition(0);
            }
            else if (gamepad1.x){
                servo.setPosition(1);
            }
            else if (gamepad1.b){
                servo.setPosition(0.5);
            }
            else if (gamepad1.y){
                servo.setPosition(0.5);
            }

            telemetry.addData("Servo Position", servo.getPosition());

        }
}
