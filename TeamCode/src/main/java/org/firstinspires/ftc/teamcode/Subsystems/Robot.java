package org.firstinspires.ftc.teamcode.Subsystems;
import com.qualcomm.robotcore.hardware.HardwareMap;

    //
    public class Robot {
        public final Drivetrain drivetrain;

        public final Intake intake;
        public final Loader loader;
        public final shooter shooter = null;
        public final C70 camera;

        // Wrist wrist = new Wrist();

        //public final BulkReader bulkReader;

        public Robot() {
            //drivetrain = new MecanumDrive(hardwareMap, startPose);
            loader = new Loader();
            intake = new Intake();
            shooter = new shooter();
            camera = new C70();
            drivetrain = new Drivetrain();

        }

        public void init(HardwareMap hardwareMap) {
            drivetrain.init(hardwareMap);
            intake.init(hardwareMap);
            shooter.init(hardwareMap);
            loader.init(hardwareMap);
            camera.init(hardwareMap);

        }

        public void score() {
            loader.setLoaderMotor();
            shooter.setShooter();
        }

        public void drive(){

        }

        public void intake(){

        }







    }

