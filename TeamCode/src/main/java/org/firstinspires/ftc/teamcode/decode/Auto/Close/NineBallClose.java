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
@Autonomous (name = "NineBallClose")
public class NineBallClose extends AbstractAuto {
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

        path.goal9Build();
    }

    @Override
    protected void onRun() {
        shootPreload();
        cycle3();
        cycle6();
        unloadRamp();

    }

    private void shootPreload() {

        robot.actionScheduler.addAction(
                new SequentialAction(
                        new ParallelAction(
                                new InstantAction(() -> robot.hoodServo.setHoodServo(0.61)),
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
                        new InstantAction(() -> robot.shooter.stop())

                )

        );
        robot.actionScheduler.runBlocking();
    }


    private void cycle3() {

        robot.actionScheduler.addAction(
                new SequentialAction(
                        new ParallelAction(
                                new InstantAction(()-> f.setMaxPower(0.59)),
                                new Actions.CallbackAction(
                                        RobotActions.intakeAction(1,4),
                                        path.intake3,0.01,0,f,"Intake3"
                                ),
                                new FollowPathAction(f,path.intake3)
                        ),
                        new ParallelAction(
                                new InstantAction(()-> f.setMaxPower(1)),
                                new Actions.CallbackAction(
                                        RobotActions.startShooter(1),
                                        path.shoot3,0.3,0,f,"Shoot3"
                                ),
                                new FollowPathAction(f,path.shoot3)
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

    private void cycle6() {

        robot.actionScheduler.addAction(
                new SequentialAction(
                        new ParallelAction(
                                new InstantAction(()-> f.setMaxPower(0.59)),
                                new Actions.CallbackAction(
                                        RobotActions.intakeAction(1, 4),
                                        path.intake6, 0.01, 0, f, "intake6"
                                ),
                                new FollowPathAction(f, path.intake6)
                        ),
                        new ParallelAction(
                                new InstantAction(()-> f.setMaxPower(1)),
                                new Actions.CallbackAction(
                                        RobotActions.startShooter(1), path.shoot6, 0.6, 0, f, "Shoot6"
                                ),
                                new FollowPathAction(f, path.shoot6)
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

    private void unloadRamp() {

        robot.actionScheduler.addAction(
                new FollowPathAction(f, path.unloadRamp)
        );

        robot.actionScheduler.runBlocking();
    }
}