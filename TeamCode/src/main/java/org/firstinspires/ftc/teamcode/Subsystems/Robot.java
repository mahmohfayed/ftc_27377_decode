package org.firstinspires.ftc.teamcode.Subsystems;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.hardware.HardwareMap;


import org.firstinspires.ftc.teamcode.pedroPathing.Constants;


public final class Robot {

    public final Follower drivetrain;
    //public final Loader loader;     // loader
    public final Shooter shooter;
    public final Intake intake;

    public final Loader loader;

    public final BulkReader bulkReader;
    public Robot(HardwareMap hardwareMap) {

        bulkReader = new BulkReader(hardwareMap);

        drivetrain = Constants.createFollower(hardwareMap);

        loader = new Loader();// loader wheel
        loader.init(hardwareMap);

        intake = new Intake(); // 2 intake wheel
        intake.init(hardwareMap);

        shooter = new Shooter(); // shooter wheel (2 motors)
        shooter.init(hardwareMap);
    }

    public void run() {
        bulkReader.bulkRead();
        drivetrain.update();
    }

    public void printTelemetry() {
        // add subsystem telemetry here later
    }
}
