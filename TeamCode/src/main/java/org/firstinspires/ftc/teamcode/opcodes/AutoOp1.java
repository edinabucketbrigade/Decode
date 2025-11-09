package org.firstinspires.ftc.teamcode.opcodes;

import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;

import org.firstinspires.ftc.teamcode.auto.autoPath1;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.BucketRobot;

import java.util.List;


@Autonomous(name = "AutoOp1", group = "Auto")
public class AutoOp1 extends CommandOpMode {
    private Follower follower;

    private autoPath1 autoPath;
    private BucketRobot robot;
    private List<LynxModule> hubs;



    @Override
    public void initialize() {
        telemetry = new JoinedTelemetry(telemetry, PanelsTelemetry.INSTANCE.getFtcTelemetry());
        super.reset();

        robot = new BucketRobot(hardwareMap);

        // Initialize follower
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose((new Pose(56.000, 8.000)));

        hubs = hardwareMap.getAll(LynxModule.class);
        hubs.forEach(hub -> hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL));

        autoPath = new autoPath1();
        autoPath.Paths(follower);



        // Create the autonomous command sequence
        SequentialCommandGroup autonomousSequence = new SequentialCommandGroup(
                // Score preload
                new FollowPathCommand(follower, autoPath.Path1),

                new WaitCommand(500),
                // Shoot the left side twice, then the right
                robot.enableOutake(),
                robot.shootPattern(),

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
        robot.run();

        telemetry.addData("Pose", "<%d,%d>:%d",
                follower.getPose().getX(),
                follower.getPose().getY(),
                follower.getPose().getHeading());
        telemetry.update();
    }
}
