package org.firstinspires.ftc.teamcode.decode.testing;


import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

    @Autonomous(name = "Pedro Pathing Autonomous", group = "Autonomous")
    @Configurable // Panels
    public class PGPBlueBottom extends OpMode {

        private TelemetryManager panelsTelemetry; // Panels Telemetry instance
        public Follower follower; // Pedro Pathing follower instance
        private int pathState; // Current autonomous path state (state machine)
        private Paths paths; // Paths defined in the Paths class

        @Override
        public void init() {
            panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

            follower = Constants.createFollower(hardwareMap);
            follower.setStartingPose(new Pose(72, 8, Math.toRadians(90)));

            paths = new Paths(follower); // Build paths

            panelsTelemetry.debug("Status", "Initialized");
            panelsTelemetry.update(telemetry);
        }

        @Override
        public void loop() {
            follower.update(); // Update Pedro Pathing
            pathState = autonomousPathUpdate(); // Update autonomous state machine

            // Log values to Panels and Driver Station
            panelsTelemetry.debug("Path State", pathState);
            panelsTelemetry.debug("X", follower.getPose().getX());
            panelsTelemetry.debug("Y", follower.getPose().getY());
            panelsTelemetry.debug("Heading", follower.getPose().getHeading());
            panelsTelemetry.update(telemetry);
        }

        public static class Paths {

            public PathChain Path1;
            public PathChain Path2;
            public PathChain Path3;
            public PathChain Path4;
            public PathChain Path5;
            public PathChain Path6;

            public Paths(Follower follower) {
                Path1 = follower
                        .pathBuilder()
                        .addPath(
                                new BezierLine(new Pose(79.886, 8.193), new Pose(72.102, 119.624))
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180))
                        .build();

                Path2 = follower
                        .pathBuilder()
                        .addPath(
                                new BezierLine(new Pose(72.102, 119.624), new Pose(47.727, 118.805))
                        )
                        .setTangentHeadingInterpolation()
                        .build();

                Path3 = follower
                        .pathBuilder()
                        .addPath(
                                new BezierCurve(
                                        new Pose(47.727, 118.805),
                                        new Pose(29.292, 54.282),
                                        new Pose(105.900, 59.198)
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(0))
                        .build();

                Path4 = follower
                        .pathBuilder()
                        .addPath(
                                new BezierLine(new Pose(105.900, 59.198), new Pose(138.469, 59.812))
                        )
                        .setTangentHeadingInterpolation()
                        .build();

                Path5 = follower
                        .pathBuilder()
                        .addPath(
                                new BezierCurve(
                                        new Pose(138.469, 59.812),
                                        new Pose(141.542, 40.353),
                                        new Pose(50.595, 44.859)
                                )
                        )
                        .setTangentHeadingInterpolation()
                        .build();

                Path6 = follower
                        .pathBuilder()
                        .addPath(
                                new BezierLine(new Pose(50.595, 44.859), new Pose(47.932, 118.600))
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(150))
                        .build();
            }
        }

        public int autonomousPathUpdate() {
            // Add your state machine Here
            // Access paths with paths.pathName
            // Refer to the Pedro Pathing Docs (Auto Example) for an example state machine
            return pathState;
        }
    }

