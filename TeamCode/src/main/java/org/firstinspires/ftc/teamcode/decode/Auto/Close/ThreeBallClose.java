package org.firstinspires.ftc.teamcode.decode.Auto.Close;

import static org.firstinspires.ftc.teamcode.decode.Subsystems.Common.robot;

import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.decode.Auto.AbstractAuto;
import org.firstinspires.ftc.teamcode.decode.Auto.Close.Paths;
import org.firstinspires.ftc.teamcode.decode.Subsystems.Actions;
import org.firstinspires.ftc.teamcode.decode.Subsystems.Common;
import org.firstinspires.ftc.teamcode.decode.Subsystems.FollowPathAction;
import org.firstinspires.ftc.teamcode.decode.Subsystems.RobotActions;
@Autonomous (name = "3BallClose")
public class ThreeBallClose extends AbstractAuto {
    private Follower f;
    private Paths path;

    @Override
    protected Pose getStartPose() {
        return Paths.P_START;
    }


    @Override
    protected void onInit() {
        f = robot.drivetrain;
        path = new Paths(f);

        if (Common.isRed != Paths.isPathRed) {
            Paths.isPathRed = !Paths.isPathRed;
            path.mirrorAll();
        }

        path.goal3Build();
    }

    @Override
    protected void onRun() {
        shootPreload();

    }

    private void shootPreload() {

        robot.actionScheduler.addAction(
                new SequentialAction(
                        new ParallelAction(
                                new InstantAction(() -> robot.hoodServo.setHoodServo(0.62)),
                                new Actions.CallbackAction(
                                        RobotActions.startShooter(1),
                                        path.shootPreload, 0.5, 0, f, "Preloadrev"
                                ),
                                new FollowPathAction(f, path.shootPreload, true)

                        ),
                        new ParallelAction(
                                RobotActions.intakeAction(1, 1),
                                RobotActions.loaderAction(1, 1)
                        ),
                        new InstantAction(() -> robot.shooter.stop()),
                        new FollowPathAction(f, path.leave)

                )

        );
        robot.actionScheduler.runBlocking();
    }
}