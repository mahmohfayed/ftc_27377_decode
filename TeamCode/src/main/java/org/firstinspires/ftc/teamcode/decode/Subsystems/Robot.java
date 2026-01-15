package org.firstinspires.ftc.teamcode.decode.Subsystems;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

public final class Robot {

    public final Follower drivetrain;
    public final Loader loader;     // loader
    public final Shooter shooter;
    public final Intake intake;


    public final BulkReader bulkReader;
    public final ActionScheduler actionScheduler;

    public Robot(HardwareMap hardwareMap) {

        bulkReader = new BulkReader(hardwareMap);

        drivetrain = Constants.createFollower(hardwareMap);
        actionScheduler = new ActionScheduler();

        loader = new Loader(hardwareMap);// loader wheel

        intake = new Intake(hardwareMap); // 2 intake wheel

        shooter = new Shooter(hardwareMap); // shooter wheel (2 motors)
    }

    public void run() {
        bulkReader.bulkRead();
        drivetrain.update();
        actionScheduler.run();
    }

    public void printTelemetry() {
        // add subsystem telemetry here later
    }
}
