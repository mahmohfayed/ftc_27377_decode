package org.firstinspires.ftc.teamcode.decode.Auto;

import static org.firstinspires.ftc.teamcode.decode.Subsystems.Common.robot;

import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.decode.Subsystems.Actions;
import org.firstinspires.ftc.teamcode.decode.Subsystems.Common;
import org.firstinspires.ftc.teamcode.decode.Subsystems.FollowPathAction;
import org.firstinspires.ftc.teamcode.decode.Subsystems.RobotActions;
// shoot positon needs to be adjusted to x: 70, y: 80
@Autonomous (name = "Red3Ball")
public class ThreeBallAuto extends AbstractAuto {
    private Follower f;
    private Paths path;
    @Override
    protected Pose getStartPose() {
        return Paths.START;
    }

    @Override
    protected void onRun() {
        shootPreload();
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

    private void shootPreload() {
        path.preload.getPath(0).setTValueConstraint(0.85);
        robot.actionScheduler.addAction(
                new SequentialAction(
                        new ParallelAction(
                                new Actions.CallbackAction(RobotActions.startShooter(0),path.preload,0.2,0,f,"RevShooterPreLoad"),
                                new FollowPathAction(f,path.preload,true)
                        ),
                        new ParallelAction(
                                RobotActions.intakeAction(1,5),
                                RobotActions.loaderAction(1,5)
                        ),
                        new FollowPathAction(f,path.leave),
                        new InstantAction(()-> robot.shooter.stop())

                )
        );

        robot.actionScheduler.runBlocking();
    }
}

