package org.firstinspires.ftc.teamcode.decode.Subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class HoodServo {

    private Servo leftServo;
    private Servo rightServo;

    public void init(HardwareMap hardwareMap) {
        //CRservo = hardwareMap.get(CRServo.class,"hoodServo");
        leftServo = hardwareMap.get(Servo.class, "leftServo");
       // rightServo = hardwareMap.get(Servo.class, "rightServo");

        //  rightServo.setDirection(Servo.Direction.REVERSE);

    }



        public void setHoodservo (double angle){
            //rightServo.setPosition(angle);
            //
            leftServo.setPosition(angle);
        }


    /*
    public void CRServo(double power, boolean isForward){
        if (isForward) { // goes down
            CRservo.setDirection(DcMotorSimple.Direction.FORWARD);
            CRservo.setPower(power);
        } else { // goes up
            CRservo.setDirection(DcMotorSimple.Direction.REVERSE);
            CRservo.setPower(power);
        }
    }

    public void CRStop(){
        CRservo.setPower(0);
    }

    public double getPower(){
        return CRservo.getPower();
    }
    */


        public double getPosition () {

            return leftServo.getPosition();
        }
    }

