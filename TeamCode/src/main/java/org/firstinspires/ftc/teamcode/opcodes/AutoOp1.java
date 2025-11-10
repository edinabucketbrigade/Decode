package org.firstinspires.ftc.teamcode.opcodes;

import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;

import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;



import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.auto.autoPath1;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.BucketRobot;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Outake;

import java.util.List;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.CommandManager;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;


@Autonomous(name = "AutoOp1", group = "Auto")
public class AutoOp1 extends NextFTCOpMode {
    public AutoOp1() {
        addComponents(
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE,
                new SubsystemComponent(BucketRobot.INSTANCE),
                new PedroComponent(Constants::createFollower)
        );

    }

    private Command autonomousSequence;

    @Override public void onWaitForStart() { }
    @Override public void onStartButtonPressed() {
        // Schedule the autonomous sequence
        autonomousSequence.schedule();
    }
    @Override public void onUpdate() {
        telemetry.addData("Pose", "<%d,%d>:%d",
                PedroComponent.follower().getPose().getX(),
                PedroComponent.follower().getPose().getY(),
                PedroComponent.follower().getPose().getHeading());
        telemetry.addData("Command", CommandManager.INSTANCE.snapshot());
        telemetry.update();

    }
    @Override public void onStop() { }

    @Override public void onInit() {
        telemetry = new JoinedTelemetry(telemetry, PanelsTelemetry.INSTANCE.getFtcTelemetry());

        PedroComponent.follower().setStartingPose((new Pose(56.000, 8.000)));

        autoPath1 autoPath = new autoPath1();
        autoPath.Paths(PedroComponent.follower());

        // Create the autonomous command sequence
        autonomousSequence = new SequentialGroup(
                // Score preload
                new FollowPath(autoPath.Path1),
                Outake.INSTANCE.on,
                //BucketRobot.INSTANCE.shootPattern,
                // Goes to the nearest artifacts  and collects them
                new FollowPath(autoPath.Path2),
                new FollowPath(autoPath.Path3)

        );

    }

}
