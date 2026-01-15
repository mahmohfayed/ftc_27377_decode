package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;




@Autonomous(name = "red side Striaght")
// moves to a defensive positon infront blue goal
public class striaghtBlue extends OpMode {

    private Follower follower;
    private Timer pathTimer, opmodeTimer;
    private int pathState;

    // --------- Step 2: Define poses ---------
    private final Pose startPose = new Pose(85.0, 8.0, Math.toRadians(90));   // robot start
    private final Pose endPose   = new Pose(85.0, 60, Math.toRadians(90));  // end point

    // --------- Step 3: Define paths ---------
    private Path simplePath;       // single BezierLine
    private PathChain simpleChain; // if you want a chain

    public void buildPaths() {
        // Straight line from start → end
        simplePath = new Path(new BezierLine(startPose, endPose));
        simplePath.setLinearHeadingInterpolation(startPose.getHeading(), endPose.getHeading());

        // Example PathChain version (optional)
        simpleChain = follower.pathBuilder()
                .addPath(new BezierLine(startPose, endPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), endPose.getHeading())
                .build();
    }

    // --------- Step 4: FSM (path state machine) ---------
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                // Follow our simple straight line
                follower.followPath(simplePath);
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
