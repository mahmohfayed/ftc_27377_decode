package org.firstinspires.ftc.teamcode.decode.Subsystems;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Shooter {

    private MotorEx shooter;
    private MotorEx followerShooter;

    // Constructor to initialize the motors
    public Shooter(HardwareMap hardwareMap) {
        // Using FTCLib MotorEx
        shooter = new MotorEx(hardwareMap, "shooter");
        followerShooter = new MotorEx(hardwareMap, "followerShooter");

        // Reset encoders
        shooter.resetEncoder();
        followerShooter.resetEncoder();

        // Set to use encoders for velocity control
        shooter.setRunMode(Motor.RunMode.VelocityControl);
        followerShooter.setRunMode(Motor.RunMode.VelocityControl);

        // Optional: Make follower mirror the main motor
        followerShooter.setInverted(true); // Adjust based on your setup
    }

    // Method to set shooter velocity
    public void setVelocity(double velocity) {
        shooter.setVelocity(velocity);
        followerShooter.setVelocity(velocity);
    }

    // Method to stop the shooter
    public void stop() {
        shooter.stopMotor();
        followerShooter.stopMotor();
    }

    // Get current velocity
    public double getVelocity() {
        return shooter.getVelocity();
    }

    // Method for telemetry
    public void displayTelemetry(Telemetry telemetry) {
        telemetry.addData("Shooter Velocity", getCurrentVelocity());
    }

    // Method for telemetry
    public double getCurrentVelocity() {
        return shooter.getVelocity();
    }
}