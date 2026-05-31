package org.firstinspires.ftc.teamcode.decode.Auto.Audience;

import static org.firstinspires.ftc.teamcode.decode.Subsystems.Common.robot;

import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.decode.Auto.AbstractAuto;
import org.firstinspires.ftc.teamcode.decode.Subsystems.Actions;
import org.firstinspires.ftc.teamcode.decode.Subsystems.Common;
import org.firstinspires.ftc.teamcode.decode.Subsystems.FollowPathAction;
import org.firstinspires.ftc.teamcode.decode.Subsystems.RobotActions;

@Autonomous(name = "AudienceHuman2")
public class AudienceHuman extends AbstractAuto {

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


        path.human6Build();
    }


    @Override
    protected void onRun() {
        shootPreload();
        Humancycle3();
        Humancycle6();
    }

    private void shootPreload() {

        robot.actionScheduler.addAction(
                new SequentialAction(
                        new ParallelAction(
                                new InstantAction(() -> robot.hoodServo.setHoodServo(0.7)),
                                new Actions.CallbackAction(
                                        RobotActions.startShooter(1.5),
                                        path.shootPreload,0.1,0,f,"Preloadrev"
                                ),
                                new FollowPathAction(f,path.shootPreload,true)

                        ),
                        new ParallelAction(
                                RobotActions.intakeAction(1,2),
                                RobotActions.loaderAction(1,2)
                        ),
                        new InstantAction(()-> robot.shooter.stop()),
                        new SleepAction(100)// change depending on teamate in milliseconds

                )

        );
        robot.actionScheduler.runBlocking();
    }


    private void Humancycle3() {

        robot.actionScheduler.addAction(
                new SequentialAction(
                        new ParallelAction(
                                new InstantAction(()-> f.setMaxPower(0.7)),
                                new Actions.CallbackAction(
                                        RobotActions.intakeAction(1, 5),
                                        path.intakeHuman, 0.01, 0, f, "IntakeHuman3"
                                ),
                                new FollowPathAction(f,path.intakeHuman)
                        ),
                        new ParallelAction(
                                new InstantAction(()-> f.setMaxPower(0.85)),
                                new Actions.CallbackAction(
                                        RobotActions.startShooter(3),
                                        path.shootHuman, 0.1, 0, f, "Shoothuman3"
                                ),
                                new FollowPathAction(f, path.shootHuman)
                        ),
                        new ParallelAction(
                                RobotActions.intakeAction(1, 1.5),
                                RobotActions.loaderAction(1, 1.5)
                        ),
                        new InstantAction(()-> robot.shooter.stop()),
                        new SleepAction(1000)// change depending on teamate


                )
        );
        robot.actionScheduler.runBlocking();
    }

    private void Humancycle6() {

        robot.actionScheduler.addAction(
                new SequentialAction(
                        new ParallelAction(
                                new InstantAction(()-> f.setMaxPower(0.7)),
                                new Actions.CallbackAction(
                                        RobotActions.intakeAction(1, 5),
                                        path.intakeHuman, 0.01, 0, f, "intakeHuman"
                                ),
                                new FollowPathAction(f, path.intakeHuman)
                        ),
                        new ParallelAction(
                                new InstantAction(()-> f.setMaxPower(0.85)),
                                new Actions.CallbackAction(
                                        RobotActions.startShooter(3), path.shootHuman, 0.5, 0, f, "shootHuman"
                                ),
                                new FollowPathAction(f, path.shootHuman)
                        ),
                        new ParallelAction(
                                RobotActions.intakeAction(1, 1.5),
                                RobotActions.loaderAction(1, 1.5)
                        ),
                        new InstantAction(()-> robot.shooter.stop())
                )
        );
        robot.actionScheduler.runBlocking();
    }

}