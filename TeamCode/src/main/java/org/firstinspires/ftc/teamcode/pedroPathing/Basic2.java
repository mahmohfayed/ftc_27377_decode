package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
@Disabled

@Autonomous(name = "Basic2", group = "TestAuto") // moves the balls up to the goal
public class Basic2 extends OpMode {

    private Follower follower;
    private Timer pathTimer, opmodeTimer;
    private int pathState;

    // --------- Step 2: Define poses ---------
    private final Pose startPose = new Pose(56.0, 8.0, Math.toRadians(90));   // robot start
    private final Pose endPose   = new Pose(56.0, 36.0, Math.toRadians(90));  // end point

    // --------- Step 3: Define paths ---------
    private Path simplePath;       // single BezierLine
    private PathChain simpleChain; // if you want a chain

    public void buildPaths() {
        // Straight line from start → end
        // simplePath = new Path(new BezierLine(startPose, endPose));
        // simplePath.setLinearHeadingInterpolation(startPose.getHeading(), endPose.getHeading());

        // Example PathChain version (optional)
        simpleChain = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(56.000, 8.000), new Pose(56.000, 36.000)))
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(0))
                .addPath(
                        new BezierCurve(
                                new Pose(56.000, 36.000),
                                new Pose(73.435, 51.320),
                                new Pose(83.564, 35.789)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-90))
                .addPath(new BezierLine(new Pose(83.564, 35.789), new Pose(83.733, 7.934)))
                .setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(180))
                .addPath(new BezierLine(new Pose(83.733, 7.934), new Pose(56.047, 8.103)))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(90))
                .build();
    }

    // --------- Step 4: FSM (path state machine) ---------
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                // Follow our simple straight line
                follower.followPath(simpleChain);
                setPathState(1);
                break;

            case 1:
                if (!follower.isBusy()) {
                    // Finished path → stop
                    setPathState(-1);
                }
                break;
        }
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    // --------- Step 5: OpMode lifecycle ---------
    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        follower = Constants.createFollower(hardwareMap);
        // If you don’t have Constants, directly construct your Follower:
        // follower = new Follower(hardwareMap);

        follower.setStartingPose(startPose);

        buildPaths();

    }

    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();

        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }

    @Override
    public void init_loop() {}

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    @Override
    public void stop() {}
}
