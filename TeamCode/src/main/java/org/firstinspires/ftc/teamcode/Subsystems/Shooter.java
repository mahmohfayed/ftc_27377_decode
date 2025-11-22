package org.firstinspires.ftc.teamcode.Subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;


import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.controls.controllers.PIDController;
import org.firstinspires.ftc.teamcode.controls.gainmatrices.PIDGains;
import org.firstinspires.ftc.teamcode.controls.motion.State;

//(name = "Shooter PID")
public class Shooter {

    MotorEx shooter, followerShooter;

    // TARGET SPEED (change this later)
    double targetRPM = 3000;

    // PID gains (start here)
    double Kp = 0.0006;
    double Ki = 0.00002;
    double Kd = 0.0001;

    private final PIDController controller = new PIDController();

    public static PIDGains gains = new PIDGains(
            0,0,0, Double.POSITIVE_INFINITY
    );


    public void init(HardwareMap hardwareMap) {
        shooter = new MotorEx(hardwareMap, "shooter", Motor.GoBILDA.BARE);
        //shooter.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        followerShooter = new MotorEx(hardwareMap, "shooter", Motor.GoBILDA.BARE);
       // followerShooter.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);


        controller.setGains(gains);


    }


    public void setShooter() {


        controller.setTarget(new State(targetRPM));


        // calculate dt
//        double currentTime = System.nanoTime() / 1e9;
//        double dt = currentTime - lastTime;
//        lastTime = currentTime;
//
//        // read velocity
//        double currentVel = shooter.getVelocity(); // ticks/sec
//
//        // convert target RPM to ticks/sec
//        double targetVel = targetRPM *
//                shooter.getMotorType().getTicksPerRev() / 60.0;
//
//        // PID calculations
//        double error = targetVel - currentVel;
//        integral += error * dt;
//        double derivative = (error - lastError) / dt;
//
//        double output = Kp * error + Ki * integral + Kd * derivative;
//
//        // clamp power
//        output = Math.min(Math.max(output, 0), 1);
//
//        // set motor power
//        shooter.setPower(output);
//        lastError = error;

     shooter.set(controller.calculate(new State(shooter.getVelocity())));
     followerShooter.set(shooter.get());

    }
    public void shootArtifacts() {
        shooter.set(1);
        followerShooter.set(1);
    }
    public void stop() {
        shooter.set(0);
    }
}
