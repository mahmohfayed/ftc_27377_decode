package org.firstinspires.ftc.teamcode.decode.testing;


import com.pedropathing.follower.Follower;
import com.pedropathing.ftc.FTCCoordinates;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@TeleOp
    public class ExampleAprilTagUsage extends OpMode {
       private Limelight3A camera; //any camera here
        private Follower follower;
        private boolean following = false;

        private static final boolean USE_WEBCAM = true;  // Set true to use a webcam, or false for a phone camera

        private AprilTagProcessor aprilTag;
        private VisionPortal visionPortal;
        private AprilTagDetection desiredTag;
        private boolean targetFound = false;

        private final Pose TARGET_LOCATION = new Pose(56.0, 36.0, Math.toRadians(90)); //Put the target location here

        @Override
        public void init() {

            initAprilTag();
            camera = hardwareMap.get(Limelight3A.class, "limelight");
            follower = Constants.createFollower(hardwareMap);
            follower.setStartingPose(new Pose()); //set your starting pose
        }

        @Override
        public void start() {
            camera.start();
        }

        @Override
        public void loop() {
            follower.update();

            //if you're not using limelight you can follow the same steps: build an offset pose, put your heading offset, and generate a path etc

            if (!following) {
                follower.followPath(
                        follower.pathBuilder()
                                .addPath(new BezierLine(follower.getPose(), TARGET_LOCATION))
                                .setLinearHeadingInterpolation(follower.getHeading(), TARGET_LOCATION.minus(follower.getPose()).getAsVector().getTheta())
                                .build()
                );
            }

            //This uses the aprilTag to relocalize your robot
            //You can also create a custom AprilTag fusion Localizer for the follower if you want to use this by default for all your autos
            follower.setPose(getRobotPoseFromCamera());

            if (following && !follower.isBusy()) following = false;
        }

        private Pose getRobotPoseFromCamera() {
            //Fill this out to get the robot Pose from the camera's output (apply any filters if you need to using follower.getPose() for fusion)
            //Pedro Pathing has built-in KalmanFilter and LowPassFilter classes you can use for this

            //Use this to convert standard FTC coordinates to standard Pedro Pathing coordinates
            return new Pose(0, 0, 0, FTCCoordinates.INSTANCE).getAsCoordinateSystem(PedroCoordinates.INSTANCE);
        }


    private void initAprilTag() {
        // Create the AprilTag processor by using a builder.
        aprilTag = new AprilTagProcessor.Builder().build();

        aprilTag.setDecimation(2);

        // Create the vision portal by using a builder.
        if (USE_WEBCAM) {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                    .addProcessor(aprilTag)
                    .build();
            } else {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(BuiltinCameraDirection.BACK)
                    .addProcessor(aprilTag)
                    .build();
        }
    }
}

