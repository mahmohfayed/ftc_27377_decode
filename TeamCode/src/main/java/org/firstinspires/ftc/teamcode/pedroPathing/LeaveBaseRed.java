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

@Autonomous(name = "LeaveBaseRed")
public class LeaveBaseRed extends OpMode {

    private Follower follower;
    private Timer pathTimer, opmodeTimer;
    private int pathState;

    // --------- Step 2: Define poses ---------
    private final Pose startPose = new Pose(72, 8, Math.toRadians(90));

    // --------- Step 3: Define paths ---------
    private PathChain simpleChain;

    public void buildPaths() {

        simpleChain = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(56.000, 8.000), new Pose(28.000, 8.000)))
                .setConstantHeadingInterpolation(Math.toRadians(90))
                .build();
    }

    // --------- Step 4: FSM ---------
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(simpleChain);
                setPathState(1);
                break;

            case 1:
                if (!follower.isBusy()) {
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