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

@Autonomous(name = "21Ball")
public class SoCalled21BallAuto extends AbstractAuto {

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
        unloadRamp();
        cycle9();
        extraOne();
        extraTwo();
        extraThree();
    }

    private void shootPreload() {

        robot.actionScheduler.addAction(
                new SequentialAction(
                        new ParallelAction(
                                new Actions.CallbackAction(
                                    RobotActions.startShooter(5),
                                    path.shootPreload,0.1,0,f,"Preloadrev"
                                ),
                                new FollowPathAction(f,path.shootPreload,true)

                        ),
                        new ParallelAction(
                                RobotActions.intakeAction(1,3),
                                RobotActions.loaderAction(1,3)
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

    private void cycle6() {

        robot.actionScheduler.addAction(
                new SequentialAction(
                        new ParallelAction(
                             new Actions.CallbackAction(
                                     RobotActions.intakeAction(1,5),
                                     path.intake6,0.1,0,f,"intake6"
                             ),
                                new FollowPathAction(f,path.intake6)
                        ),
                        new ParallelAction(
                                new Actions.CallbackAction(
                                        RobotActions.startShooter(5),path.shoot6,0.1,0,f,"Shoot6"
                                ),
                                new FollowPathAction(f,path.shoot6)
                        ),
                        new ParallelAction(
                                RobotActions.intakeAction(1,3),
                                RobotActions.loaderAction(1,3)
                        )
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

        robot.actionScheduler.addAction(
                new SequentialAction(
                        new ParallelAction(
                                new Actions.CallbackAction(
                                        RobotActions.intakeAction(1,3),path.intake9,0.1,0,f,"Intake9"
                                ),
                                new FollowPathAction(f,path.intake9)
                        ),
                        new ParallelAction(
                                new Actions.CallbackAction(
                                        RobotActions.startShooter(0.35),path.shoot9,0.1,0,f,"Shoot9"
                                ),
                                new FollowPathAction(f,path.shoot9)
                        ),
                        new ParallelAction(
                                RobotActions.intakeAction(1,3),
                                RobotActions.loaderAction(1,3)
                        ),
                        new FollowPathAction(f,path.leave)

                )
        );

        robot.actionScheduler.runBlocking();
    }




    private void extraOne() {

        robot.actionScheduler.addAction(
                new SequentialAction(
                        new ParallelAction(
                                new Actions.CallbackAction(
                                        RobotActions.intakeAction(1,5),path.intakeExtra1,0.1,0,f,"IntakeExtra1"
                                ),
                                new FollowPathAction(f,path.intakeExtra1)
                        ),
                                new ParallelAction(
                                        new Actions.CallbackAction(
                                                RobotActions.startShooter(5),path.shootExtra1,0.1,0,f,"ShootExtra1"
                                        ),
                                        new FollowPathAction(f,path.shootExtra1)
                                ),
                                new ParallelAction(
                                        RobotActions.intakeAction(1,3),
                                        RobotActions.intakeAction(1,3)
                                )

                )
        );

        robot.actionScheduler.runBlocking();
    }
    private void extraTwo() {

        robot.actionScheduler.addAction(
                new SequentialAction(
                        new ParallelAction(
                                new Actions.CallbackAction(
                                        RobotActions.intakeAction(1,5),path.intakeExtra2,0.1,0,f,"IntakeExtra2"
                                ),
                                new FollowPathAction(f,path.intakeExtra2)
                        ),
                                new ParallelAction(
                                        new Actions.CallbackAction(
                                                RobotActions.startShooter(5),path.shootExtra2,0.1,0,f,"ShootExtra2"
                                        ),
                                        new FollowPathAction(f,path.shootExtra2)
                                ),
                                new ParallelAction(
                                        RobotActions.intakeAction(1,3),
                                        RobotActions.intakeAction(1,3)
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
                                        RobotActions.intakeAction(1,5),path.intakeExtra3,0.1,0,f,"IntakeExtra3"
                                ),
                                new FollowPathAction(f,path.intakeExtra3)
                        ),
                                new ParallelAction(
                                        new Actions.CallbackAction(
                                                RobotActions.startShooter(5),path.shootExtra3,0.1,0,f,"ShootExtra3"
                                        ),
                                        new FollowPathAction(f,path.shootExtra3)
                                ),
                                new ParallelAction(
                                        RobotActions.intakeAction(1,3),
                                        RobotActions.intakeAction(1,3)
                                ),
                                new FollowPathAction(f,path.leave)

                )
        );

        robot.actionScheduler.runBlocking();
    }


}

