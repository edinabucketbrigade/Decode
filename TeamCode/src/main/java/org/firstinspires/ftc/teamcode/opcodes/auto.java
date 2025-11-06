package org.firstinspires.ftc.teamcode.opcodes;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;


import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Outake;


@Autonomous
public class auto extends CommandOpMode {
    private Follower follower;
    private autoPath1 autoPath;
    private Outake outake;


    // Mechanism commands - replace these with your actual subsystem commands


    @Override
    public void initialize() {
        super.reset();

        // Initialize follower
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose((new Pose(56.000, 8.000)));

        autoPath = new autoPath1();
        autoPath.Paths(follower);

        outake = new Outake(hardwareMap);

        // Create the autonomous command sequence
        SequentialCommandGroup autonomousSequence = new SequentialCommandGroup(
                // Score preload
                new FollowPathCommand(follower, autoPath.Path1),

                new WaitCommand(500),
                // Shoot the left side twice, then the right
                new InstantCommand(() -> outake.StartOutake()),
                new WaitCommand(500),
                outake.shootL(),
                new WaitCommand(1000),
                outake.shootR(),
                new WaitCommand(500),
                new InstantCommand(() -> outake.SettriggerR(Outake.triggerPosition)),
                new WaitCommand(100),
                new InstantCommand(() -> outake.SettriggerR(Outake.resetPosition)),

                // Goes to the nearest artifacts  and collects them
                new FollowPathCommand(follower, autoPath.Path2),


                new FollowPathCommand(follower, autoPath.Path3)



        );

        // Schedule the autonomous sequence
        schedule(autonomousSequence);
    }

    @Override
    public void run() {
        super.run();


    }
}
