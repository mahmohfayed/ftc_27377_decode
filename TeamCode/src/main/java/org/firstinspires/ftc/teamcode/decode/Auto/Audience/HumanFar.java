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

@Autonomous(name = "Humanfar")
public class HumanFar extends AbstractAuto {

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


        path.goal21Build();
    }


    @Override
    protected void onRun() {
        shootPreload();
        cycle3();
        cycle6();
        Humancycle3();

    }

    private void shootPreload() {

        robot.actionScheduler.addAction(
                new SequentialAction(
                        new ParallelAction(
                                new InstantAction(() -> f.setMaxPower(0.95)),

                                new Actions.CallbackAction(
                                        RobotActions.startShooter(2.5),
                                        path.shootPreload, 0.2, 0, f, "Preloadrev"
                                ),
                                new FollowPathAction(f, path.shootPreload, true)

                        ),
                        new ParallelAction(
                                RobotActions.intakeAction(1, 1),
                                RobotActions.loaderAction(1, 1)
                        ),
                        new InstantAction(() -> robot.shooter.stop())

                )

        );
        robot.actionScheduler.runBlocking();
    }


    private void cycle3() {
//    path.shoot3.getPath(0).setHeadingConstraint(0.0349);
        path.shoot3.getPath(0).setBrakingStart(0.7);
        path.shoot3.getPath(0).setBrakingStrength(0.7);
        robot.actionScheduler.addAction(
                new SequentialAction(
                        new ParallelAction(
                                new InstantAction(() -> f.setMaxPower(0.75)),
                                new Actions.CallbackAction(
                                        new ParallelAction(RobotActions.intakeAction(1, 3)
//                                                RobotActions.loaderAction(1, 1.5)
                                        ),
                                        path.intake3, 0.1, 0, f, "Intake3"
                                ),
                                new FollowPathAction(f, path.intake3)
                        ),
                        new ParallelAction(
                                new InstantAction(() -> f.setMaxPower(0.75)),
                                new Actions.CallbackAction(
                                        RobotActions.startShooter(2),
                                        path.shoot3, 0.2, 0, f, "Shoot3"
                                ),
                                new FollowPathAction(f, path.shoot3)
                        ),
                        new ParallelAction(
                                RobotActions.intakeAction(1, 1),
                                RobotActions.loaderAction(1, 1)
                        ),
                        new InstantAction(() -> robot.shooter.stop())


                )
        );
        robot.actionScheduler.runBlocking();
    }
    private void cycle6() {
        path.shoot6.getPath(0).setBrakingStart(0.7);
        path.shoot6.getPath(0).setBrakingStrength(0.7);

        robot.actionScheduler.addAction(
                new SequentialAction(
                        new ParallelAction(
                                new InstantAction(()-> f.setMaxPower(0.85)),
                                new Actions.CallbackAction(
                                        RobotActions.intakeAction(1,3),
                                        path.intake6,0.1,0,f,"intake6"
                                ),
                                new FollowPathAction(f,path.intake6)
                        ),
                        new ParallelAction(
                                new InstantAction(()-> f.setMaxPower(1)),
                                new Actions.CallbackAction(
                                        RobotActions.startShooter(1.5),path.shoot6,0.3,0,f,"Shoot6"
                                ),
                                new FollowPathAction(f,path.shoot6)
                        ),
                        new ParallelAction(
                                RobotActions.intakeAction(1,1),
                                RobotActions.loaderAction(1,1)
                        ),
                        new InstantAction(()-> robot.shooter.stop())

                )
        );
        robot.actionScheduler.runBlocking();
    }

    private void Humancycle3() {

        robot.actionScheduler.addAction(
                new SequentialAction(
                        new ParallelAction(
                                new InstantAction(() -> f.setMaxPower(0.7)),
                                new Actions.CallbackAction(
                                        RobotActions.intakeAction(1, 5),
                                        path.intakeHuman, 0.1, 0, f, "IntakeHuman3"
                                ),
                                new FollowPathAction(f, path.intakeHuman)
                        ),
                        new ParallelAction(
                                new InstantAction(() -> f.setMaxPower(0.85)),
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
                        new InstantAction(() -> robot.shooter.stop()),
                        new SleepAction(1000)// change depending on teamate


                )
        );
        robot.actionScheduler.runBlocking();
    }
}