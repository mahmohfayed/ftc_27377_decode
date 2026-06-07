package org.firstinspires.ftc.teamcode.decode.Auto.Close;

import static org.firstinspires.ftc.teamcode.decode.Subsystems.Common.robot;

import com.acmerobotics.roadrunner.Action;
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

@Autonomous (name = "21BallClose")
public class Ilove21balls extends AbstractAuto {
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

        // Tell RobotActions which goal to use for distance calculations
        RobotActions.setGoal(Common.isRed ? new Pose(136, 136) : new Pose(136, 136).mirror());

        path.goal21Build();
    }

    @Override
    protected void onRun() {
        shootPreload();
        cycle6();
        cycle3();
        unloadRamp();
        cycle9();
        Human();
        extraOne();
        extraTwo();
        extraThree();
    }

    private void shootPreload() {
        robot.actionScheduler.addAction(
                new SequentialAction(
                        new InstantAction(()-> f.setMaxPower(0.85)),
                        new ParallelAction(
                                // startShooter now sets velocity + hood based on distance
                                // (same math as teleop) instead of raw full power
                                new Actions.CallbackAction(
                                        RobotActions.startShooter(10),
                                        path.shootPreload,0.01,0,f,"Preloadrev"
                                ),
                                new FollowPathAction(f,path.shootPreload,true)
                        ),
                        new ParallelAction(
                                RobotActions.intakeAction(1,1.5),
                                RobotActions.loaderAction(1,1.5)
                        ),
                        new InstantAction(()-> robot.shooter.stop()),
                        new InstantAction(()-> f.setMaxPower(1))

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
                                new InstantAction(()-> f.setMaxPower(0.7)),
                                new Actions.CallbackAction(
                                        RobotActions.intakeAction(1, 4),
                                        path.intake6, 0.01, 0, f, "intake6"
                                ),
                                new FollowPathAction(f, path.intake6)
                        ),
                        new ParallelAction(
                                new InstantAction(()-> f.setMaxPower(1)),
                                new Actions.CallbackAction(
                                        RobotActions.startShooter(1.5), path.shoot6, 0.001, 0, f, "Shoot6"
                                ),
                                new FollowPathAction(f, path.shoot6)
                        ),
                        new ParallelAction(
                                RobotActions.intakeAction(1,1.5),
                                RobotActions.loaderAction(1,1.5)
                        ),
                        new InstantAction(() -> robot.shooter.stop())
                )
        );
        robot.actionScheduler.runBlocking();
    }

    private void cycle3() {
        path.shoot3.getPath(0).setBrakingStart(0.9);
        path.shoot3.getPath(0).setBrakingStrength(0.7);
        robot.actionScheduler.addAction(
                new SequentialAction(
                        new ParallelAction(
                                new InstantAction(()-> f.setMaxPower(0.7)),
                                new Actions.CallbackAction(
                                        RobotActions.intakeAction(1,3),
                                        path.intake3,0.01,0,f,"Intake3"
                                ),
                                new FollowPathAction(f,path.intake3)
                        ),
                        new ParallelAction(
                                new InstantAction(()-> f.setMaxPower(0.8)),
                                new Actions.CallbackAction(
                                        RobotActions.startShooter(1.5),
                                        path.shoot3,0.3,0,f,"Shoot3"
                                ),
                                new FollowPathAction(f,path.shoot3)
                        ),
                        new ParallelAction(
                                RobotActions.intakeAction(1,1.5),
                                RobotActions.loaderAction(1,1.5)
                        ),
                        new InstantAction(()-> robot.shooter.stop())
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

    private void cycle9() {
        path.shoot9.getPath(0).setBrakingStart(0.8);
        path.shoot9.getPath(0).setBrakingStrength(0.8);
        robot.actionScheduler.addAction(
                new SequentialAction(
                        new ParallelAction(
                                new InstantAction(()-> f.setMaxPower(0.7)),
                                new Actions.CallbackAction(
                                        RobotActions.intakeAction(1,4),path.intake9,0.1,0,f,"Intake9"
                                ),
                                new FollowPathAction(f,path.intake9)
                        ),
                        new ParallelAction(
                                new InstantAction(()-> f.setMaxPower(1)),
                                new Actions.CallbackAction(
                                        RobotActions.startShooter(1.5),path.shoot9,0.001,0,f,"Shoot9"
                                ),
                                new FollowPathAction(f,path.shoot9)
                        ),
                        new ParallelAction(
                                RobotActions.intakeAction(1,1.5),
                                RobotActions.loaderAction(1,1.5)
                        ),
                        new InstantAction(()-> robot.shooter.stop())
                )
        );
        robot.actionScheduler.runBlocking();
    }

    private void Human() {
        robot.actionScheduler.addAction(
                new SequentialAction(
                        new ParallelAction(
                                new InstantAction(()-> f.setMaxPower(0.7)),
                                new Actions.CallbackAction(
                                        RobotActions.intakeAction(1,4),path.intakeHuman,0.1,0,f,"IntakeHuman"
                                ),
                                new FollowPathAction(f,path.intakeHuman)
                        ),
                        new ParallelAction(
                                new InstantAction(()-> f.setMaxPower(0.85)),
                                new Actions.CallbackAction(
                                        RobotActions.startShooter(1.5),path.shootHuman,0.001,0,f,"ShootHuman"
                                ),
                                new FollowPathAction(f,path.shootHuman)
                        ),
                        new ParallelAction(
                                RobotActions.intakeAction(1,1.5),
                                RobotActions.loaderAction(1,1.5)
                        ),
                        new InstantAction(()-> robot.shooter.stop())
                )
        );
        robot.actionScheduler.runBlocking();
    }

    private void extraOne() {
        robot.actionScheduler.addAction(
                new SequentialAction(
                        new ParallelAction(
                                new Actions.CallbackAction(
                                        RobotActions.intakeAction(1,2),path.intakeExtra1,0.1,0,f,"IntakeExtra1"
                                ),
                                new FollowPathAction(f,path.intakeExtra1)
                        ),
                        new ParallelAction(
                                new Actions.CallbackAction(
                                        RobotActions.startShooter(1),path.shootExtra1,0.5,0,f,"ShootExtra1"
                                ),
                                new FollowPathAction(f,path.shootExtra1)
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

    private void extraTwo() {
        robot.actionScheduler.addAction(
                new SequentialAction(
                        new ParallelAction(
                                new Actions.CallbackAction(
                                        RobotActions.intakeAction(1,2),path.intakeExtra2,0.1,0,f,"IntakeExtra2"
                                ),
                                new FollowPathAction(f,path.intakeExtra2)
                        ),
                        new ParallelAction(
                                new Actions.CallbackAction(
                                        RobotActions.startShooter(1),path.shootExtra2,0.5,0,f,"ShootExtra2"
                                ),
                                new FollowPathAction(f,path.shootExtra2)
                        ),
                        new ParallelAction(
                                RobotActions.intakeAction(1,0.7),
                                RobotActions.loaderAction(1,0.7)
                        )
                )
        );
        robot.actionScheduler.runBlocking();
    }

    private void extraThree() {
        robot.actionScheduler.addAction(
                new SequentialAction(
                        new ParallelAction(
                                new Actions.CallbackAction(
                                        RobotActions.intakeAction(1,2),path.intakeExtra3,0.1,0,f,"IntakeExtra3"
                                ),
                                new FollowPathAction(f,path.intakeExtra3)
                        ),
                        new ParallelAction(
                                new Actions.CallbackAction(
                                        RobotActions.startShooter(1),path.shootExtra3,0.5,0,f,"ShootExtra3"
                                ),
                                new FollowPathAction(f,path.shootExtra3)
                        ),
                        new ParallelAction(
                                RobotActions.intakeAction(1,0.7),
                                RobotActions.loaderAction(1,0.7)
                        ),
                        new FollowPathAction(f,path.leave)
                )
        );
        robot.actionScheduler.runBlocking();
    }
}
