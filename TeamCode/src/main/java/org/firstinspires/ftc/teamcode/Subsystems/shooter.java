package org.firstinspires.ftc.teamcode.Subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

//(name = "Shooter PID")
public class shooter {

    DcMotorEx shooter;

    // TARGET SPEED (change this later)
    double targetRPM = 3000;

    // PID gains (start here)
    double Kp = 0.0006;
    double Ki = 0.00002;
    double Kd = 0.0001;

    double integral = 0;
    double lastError = 0;
    double lastTime = 0;

    public void init(HardwareMap hardwareMap) {
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        shooter.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        lastTime = System.nanoTime() / 1e9;
    }


    public void setShooter() {

        // calculate dt
        double currentTime = System.nanoTime() / 1e9;
        double dt = currentTime - lastTime;
        lastTime = currentTime;

        // read velocity
        double currentVel = shooter.getVelocity(); // ticks/sec

        // convert target RPM to ticks/sec
        double targetVel = targetRPM *
                shooter.getMotorType().getTicksPerRev() / 60.0;

        // PID calculations
        double error = targetVel - currentVel;
        integral += error * dt;
        double derivative = (error - lastError) / dt;

        double output = Kp * error + Ki * integral + Kd * derivative;

        // clamp power
        output = Math.min(Math.max(output, 0), 1);

        // set motor power
        shooter.setPower(output);
        lastError = error;

        // show telemetry

//        public void shooterTelmetery(){
//            System.out.println("Velocity (ticks/sec)" + currentVel);
//            System.out.println("Error"+ error);
//            System.out.println("Power" + output);
//
//        }
    }
}
