package org.firstinspires.ftc.teamcode.decode.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public class Shooter {

    private MotorEx shooter;
    private MotorEx followerShooter;

    // Tune these in FTC Dashboard
    public static double kP = 50.0;
    public static double kI = 0.0;
    public static double kD = 0.0;
    public static double kF = 12.0;

    public Shooter(HardwareMap hardwareMap) {
        shooter         = new MotorEx(hardwareMap, "leftShooterMotor",  Motor.GoBILDA.BARE);
        followerShooter = new MotorEx(hardwareMap, "rightShooterMotor", Motor.GoBILDA.BARE);

        followerShooter.motor.setDirection(DcMotorSimple.Direction.FORWARD);
        shooter.motor.setDirection(DcMotorSimple.Direction.REVERSE);

        shooter.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        followerShooter.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);

        shooter.resetEncoder();
        followerShooter.resetEncoder();

        // Set velocity PIDF on both motors
        updatePIDF();
    }

    // Called from TeleOp every loop so dashboard changes apply live
    public void updatePIDF() {
        shooter.motorEx.setPIDFCoefficients(
                com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER,
                new com.qualcomm.robotcore.hardware.PIDFCoefficients(kP, kI, kD, kF)
        );
        followerShooter.motorEx.setPIDFCoefficients(
                com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER,
                new com.qualcomm.robotcore.hardware.PIDFCoefficients(kP, kI, kD, kF)
        );
    }

    // Raw power — used in auto
    public void shootArtifacts() {
        shooter.set(1);
        followerShooter.set(1);
    }

    // Variable velocity — used in TeleOp with distance-based control
    public void setVelocity(double velocity) {
        shooter.setVelocity(velocity);
        followerShooter.setVelocity(velocity);
    }

    public double getVelocity() {
        // Average both motors for telemetry
        return (shooter.getVelocity() + followerShooter.getVelocity()) / 2.0;
    }

    public void stop() {
        shooter.set(0);
        followerShooter.set(0);
    }
}