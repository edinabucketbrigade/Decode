package org.firstinspires.ftc.teamcode.opcodes;

import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;

import com.qualcomm.hardware.lynx.LynxModule;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;
import com.seattlesolvers.solverslib.util.TelemetryData;


import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.auto.autoPath1;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Outake;

import java.util.List;


@Autonomous(name = "AutoOp1", group = "Auto")
public class AutoOp1 extends CommandOpMode {
    private Follower follower;

    private autoPath1 autoPath;
    private Outake outake;
    private Intake intake;
    private List<LynxModule> hubs;
    Telemetry bTelemetry;



    @Override
    public void initialize() {
        bTelemetry = new JoinedTelemetry(telemetry, PanelsTelemetry.INSTANCE.getFtcTelemetry());
        super.reset();

        // Initialize follower
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose((new Pose(56.000, 8.000)));

        hubs = hardwareMap.getAll(LynxModule.class);
        hubs.forEach(hub -> hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL));

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
        hubs.forEach(LynxModule::clearBulkCache);
        super.run();

        bTelemetry.addData("X", follower.getPose().getX());
        bTelemetry.addData("Y", follower.getPose().getY());
        bTelemetry.addData("Heading", follower.getPose().getHeading());
        bTelemetry.update();
    }
}
