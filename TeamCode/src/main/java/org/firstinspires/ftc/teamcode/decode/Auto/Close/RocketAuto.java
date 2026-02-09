package org.firstinspires.ftc.teamcode.decode.Auto.Close;

import static org.firstinspires.ftc.teamcode.decode.Subsystems.Common.robot;

import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.decode.Auto.AbstractAuto;
import org.firstinspires.ftc.teamcode.decode.Auto.Close.Paths;
import org.firstinspires.ftc.teamcode.decode.Subsystems.Actions;
import org.firstinspires.ftc.teamcode.decode.Subsystems.Common;
import org.firstinspires.ftc.teamcode.decode.Subsystems.FollowPathAction;
import org.firstinspires.ftc.teamcode.decode.Subsystems.RobotActions;

@Autonomous (name = "RocketAuto")
public class RocketAuto extends AbstractAuto {
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

        path.goalRocketBuild();
    }

    @Override
    protected void onRun() {
        shootPreload();
        cycle6();
        unloadRamp();
        cycle3();

    }

    private void shootPreload() {

        robot.actionScheduler.addAction(
                new SequentialAction(
                        new ParallelAction(
                                new Actions.CallbackAction(
                                        RobotActions.startShooter(1),
                                        path.shootPreload, 0.3, 0, f, "Preloadrev"
                                ),
                                new FollowPathAction(f, path.shootPreload, true)

                        ),
                        new ParallelAction(
                                RobotActions.intakeAction(1, 1.5),
                                RobotActions.loaderAction(1, 1.5)
                        ),
                        new InstantAction(() -> robot.shooter.stop())

                )

        );
        robot.actionScheduler.runBlocking();
    }


    private void cycle6() {
        path.shoot6.getPath(0).setBrakingStart(0.8);
        path.shoot6.getPath(0).setBrakingStrength(0.8);
        robot.actionScheduler.addAction(
                new SequentialAction(
                        new ParallelAction(
                                new InstantAction(() -> f.setMaxPower(0.7)),
                                new Actions.CallbackAction(
                                        RobotActions.intakeAction(1, 4),
                                        path.intake6, 0.01, 0, f, "intake6"
                                ),
                                new FollowPathAction(f, path.intake6)
                        ),
                        new ParallelAction(
                                new InstantAction(() -> f.setMaxPower(1)),
                                new Actions.CallbackAction(
                                        RobotActions.startShooter(1), path.shoot6, 0.4, 0, f, "Shoot6"
                                ),
                                new FollowPathAction(f, path.shoot6)
                        ),
                        new ParallelAction(
                                RobotActions.intakeAction(1, 1.5),
                                RobotActions.loaderAction(1, 1.5)
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

    private void cycle3() {
        path.shoot3.getPath(0).setBrakingStart(0.9);
        path.shoot3.getPath(0).setBrakingStrength(0.7);
        robot.actionScheduler.addAction(
                new SequentialAction(
                        new ParallelAction(
                                new InstantAction(() -> f.setMaxPower(0.7)),
                                new Actions.CallbackAction(
                                        RobotActions.intakeAction(1, 3),
                                        path.intake3, 0.01, 0, f, "Intake3"
                                ),
                                new FollowPathAction(f, path.intake3)
                        ),
                        new ParallelAction(
                                new InstantAction(() -> f.setMaxPower(0.8)),
                                new Actions.CallbackAction(
                                        RobotActions.startShooter(1),
                                        path.shoot3, 0.3, 0, f, "Shoot3"
                                ),
                                new FollowPathAction(f, path.shoot3)
                        ),
                        new ParallelAction(
                                RobotActions.intakeAction(1, 1.5),
                                RobotActions.loaderAction(1, 1.5)
                        ),
                        new InstantAction(() -> robot.shooter.stop())
                )

        );
        robot.actionScheduler.runBlocking();
    }
}