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

@Autonomous(name = "AudienceSixBall")
public class AudienceSixBall extends AbstractAuto {

    private Follower f;
    private AudiencePath path;

    @Override
    protected Pose getStartPose() {
        return AudiencePath.P_START;
    }

    @Override
    protected void onInit() {

        f = robot.drivetrain;
        path = new AudiencePath(f);


        if (Common.isRed != AudiencePath.isPathRed) {
            AudiencePath.isPathRed = !AudiencePath.isPathRed;
            path.audienceMirrorAll();
        }


        path.goal6Build();
    }


    @Override
    protected void onRun() {
        shootPreload();
        cycle3();
    }

    private void shootPreload() {

        robot.actionScheduler.addAction(
                new SequentialAction(
                        new ParallelAction(
                                new Actions.CallbackAction(
                                        RobotActions.startShooter(5),
                                        path.shootPreload, 0.1, 0, f, "Preloadrev"
                                ),
                                new FollowPathAction(f, path.shootPreload, true)

                        ),
                        new ParallelAction(
                                RobotActions.intakeAction(1, 3),
                                RobotActions.loaderAction(1, 3)
                        )
                )

        );
        robot.actionScheduler.runBlocking();
    }


    private void cycle3() {

        robot.actionScheduler.addAction(
                new SequentialAction(
                        new ParallelAction(
                                new Actions.CallbackAction(
                                        RobotActions.intakeAction(1, 5),
                                        path.intake3, 0.1, 0, f, "Intake3"
                                ),
                                new FollowPathAction(f,path.intake3)
                        ),
                                new ParallelAction(
                                        new Actions.CallbackAction(
                                                RobotActions.startShooter(5),
                                                path.shoot3, 0.1, 0, f, "Shoot3"
                                        ),
                                        new FollowPathAction(f, path.shoot3)
                                ),
                                new ParallelAction(
                                        RobotActions.intakeAction(1, 3),
                                        RobotActions.loaderAction(1, 3)
                                ),
                                new FollowPathAction(f,path.leave)


                )
        );
        robot.actionScheduler.runBlocking();
    }

}