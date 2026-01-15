package org.firstinspires.ftc.teamcode.decode.Auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

public class Paths {

    private final Follower f;

    public PathChain preload;
    public PathChain firstShoot;
    public PathChain secondShoot;
    public PathChain leave;

    // -----------------------------
    // POSE VARIABLES
    // -----------------------------


    public static Pose START = new Pose(112.094, 138.091, Math.toRadians(0));

    public static Pose PRELOAD_END   = new Pose(89.135, 101.796);

    public static Pose FIRST_CURVE_P1 = new Pose(89.135, 101.796);
    public static Pose FIRST_CURVE_P2 = new Pose(79.512, 88.291);
    public static Pose FIRST_CURVE_P3 = new Pose(95.550, 83.057);

    public static Pose FIRST_LINE_END = new Pose(128.97538100820634, 83.56389214536928);

    public static Pose LEAVE_END = new Pose(88.628, 68.539);

    public static Pose SECOND_LINE_START = new Pose(120.028, 83.226);
    public static Pose SECOND_LINE_END   = new Pose(95.550, 83.057);

    public static Pose SECOND_CURVE_P2 = new Pose(79.512, 88.460);
    public static Pose SECOND_CURVE_P3 = new Pose(89.135, 101.796);

    // -----------------------------
    // HEADING VARIABLES
    // -----------------------------

    public static double HEADING_START = Math.toRadians(0);
    public static double HEADING_TO_SHOOT = Math.toRadians(38);
    public static double HEADING_STRAIGHT = Math.toRadians(0);

    public Paths(Follower follower) {
        f = follower;
    }
    public double mirrorAngleRad(double angle) {
        return Math.PI - angle;
    }
    public static boolean isPathRed = true;
    public void mirrorAll() {
        START = START.mirror();
        PRELOAD_END = PRELOAD_END.mirror();
        FIRST_CURVE_P1 = FIRST_CURVE_P1.mirror();
        FIRST_CURVE_P2 = FIRST_CURVE_P2.mirror();
        FIRST_CURVE_P3 = FIRST_CURVE_P3.mirror();
        FIRST_LINE_END = FIRST_LINE_END.mirror();
        LEAVE_END = LEAVE_END.mirror();
        SECOND_LINE_START = SECOND_LINE_START.mirror();
        SECOND_LINE_END = SECOND_LINE_END.mirror();
        SECOND_CURVE_P2 = SECOND_CURVE_P2.mirror();
        SECOND_CURVE_P3 = SECOND_CURVE_P3.mirror();

        HEADING_START = mirrorAngleRad(HEADING_START);
        HEADING_TO_SHOOT = mirrorAngleRad(HEADING_TO_SHOOT);
        HEADING_STRAIGHT = mirrorAngleRad(HEADING_STRAIGHT);
    }

    // -----------------------------------
    // GOAL 3 ROUTE
    // -----------------------------------
    public void goal3Build() {

        preload = f
                .pathBuilder()
                .addPath(new BezierLine(START, PRELOAD_END))
                .setLinearHeadingInterpolation(HEADING_START, HEADING_TO_SHOOT)
                .build();

        firstShoot = f
                .pathBuilder()
                .addPath(new BezierCurve(
                        FIRST_CURVE_P1,
                        FIRST_CURVE_P2,
                        FIRST_CURVE_P3
                ))
                .setLinearHeadingInterpolation(HEADING_TO_SHOOT, HEADING_STRAIGHT)
                .addPath(new BezierLine(FIRST_CURVE_P3, FIRST_LINE_END))
                .setConstantHeadingInterpolation(HEADING_STRAIGHT)
                .build();

        leave = f
                .pathBuilder()
                .addPath(new BezierLine(FIRST_CURVE_P1, LEAVE_END))
                .setConstantHeadingInterpolation(HEADING_TO_SHOOT)
                .build();
    }

    // -----------------------------------
    // GOAL 6 ROUTE
    // -----------------------------------
    public void goal6Build() {

        secondShoot = f
                .pathBuilder()
                .addPath(new BezierLine(SECOND_LINE_START, SECOND_LINE_END))
                .setConstantHeadingInterpolation(HEADING_STRAIGHT)
                .setReversed()
                .addPath(new BezierCurve(
                        SECOND_LINE_END,
                        SECOND_CURVE_P2,
                        SECOND_CURVE_P3
                ))
                .setLinearHeadingInterpolation(HEADING_STRAIGHT, HEADING_TO_SHOOT)
                .build();

        preload = f
                .pathBuilder()
                .addPath(new BezierLine(START, PRELOAD_END))
                .setLinearHeadingInterpolation(HEADING_START, HEADING_TO_SHOOT)
                .build();

        firstShoot = f
                .pathBuilder()
                .addPath(new BezierCurve(
                        FIRST_CURVE_P1,
                        FIRST_CURVE_P2,
                        FIRST_CURVE_P3
                ))
                .setLinearHeadingInterpolation(HEADING_TO_SHOOT, HEADING_STRAIGHT)
                .addPath(new BezierLine(FIRST_CURVE_P3, FIRST_LINE_END))
                .setConstantHeadingInterpolation(HEADING_STRAIGHT)
                .addPath(new BezierLine(FIRST_LINE_END, FIRST_CURVE_P3))
                .setConstantHeadingInterpolation(HEADING_STRAIGHT)
                .addPath(new BezierCurve(
                        FIRST_CURVE_P3,
                        FIRST_CURVE_P2,
                        FIRST_CURVE_P1
                ))
                .setLinearHeadingInterpolation(HEADING_STRAIGHT, HEADING_TO_SHOOT)
                .build();

        leave = f
                .pathBuilder()
                .addPath(new BezierLine(FIRST_CURVE_P1, LEAVE_END))
                .setConstantHeadingInterpolation(HEADING_TO_SHOOT)
                .build();
    }
}
