package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Shooter {

    private MotorEx shooter;
    private MotorEx followerShooter;

    private static final double SHOOT_VELOCITY = 2600;




    public void init(HardwareMap hardwareMap) {
        shooter = new MotorEx(hardwareMap, "leftShooterMotor", Motor.GoBILDA.BARE);
        followerShooter = new MotorEx(hardwareMap, "rightShooterMotor", Motor.GoBILDA.BARE);

        shooter.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        followerShooter.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);

        shooter.resetEncoder();
        followerShooter.resetEncoder();
    }

    public void shootArtifacts() {
        shooter.setVelocity(SHOOT_VELOCITY);
        followerShooter.setVelocity(SHOOT_VELOCITY);

    }

    public void getVelocity(){
        shooter.getVelocity();
    }

    public void stop() {
        shooter.set(0);
        followerShooter.set(0);
    }



}
