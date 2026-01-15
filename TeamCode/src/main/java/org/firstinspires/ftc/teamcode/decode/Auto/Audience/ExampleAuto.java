package org.firstinspires.ftc.teamcode.decode.Auto.Audience;

import static org.firstinspires.ftc.teamcode.decode.Subsystems.Common.robot;

import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.decode.Auto.AbstractAuto;
import org.firstinspires.ftc.teamcode.decode.Subsystems.Actions;
import org.firstinspires.ftc.teamcode.decode.Subsystems.Common;
import org.firstinspires.ftc.teamcode.decode.Subsystems.FollowPathAction;
import org.firstinspires.ftc.teamcode.decode.Subsystems.RobotActions;

@Autonomous(name = "ExampleAuto")

public class ExampleAuto extends AbstractAuto {

    private Follower f;
    private AudiencePath path;

    @Override
    protected Pose getStartPose() {return AudiencePath.P_START;}

    @Override

    protected void onInit() {
        f = robot.drivetrain;
        path = new AudiencePath(f);


        if (Common.isRed != AudiencePath.isPathRed) {
            AudiencePath.isPathRed = !AudiencePath.isPathRed;
            path.audienceMirrorAll();
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
                                new Actions.CallbackAction(
                                        RobotActions.startShooter(5),
                                        path.shootPreload, 0.2, 0, f, "preloadrev"
                                ),
                                new FollowPathAction(f, path.shootPreload, true)
                        ),
                        new ParallelAction(
                                RobotActions.loaderAction(1, 3.141592560571),
                                RobotActions.intakeAction(1, 3)
                        ),
                        new InstantAction(()-> robot.shooter.stop())
                )
        );
        robot.actionScheduler.runBlocking();
    }



}
