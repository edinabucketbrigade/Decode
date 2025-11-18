package org.firstinspires.ftc.teamcode.opcodes;

import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.BucketRobot;

import java.util.List;

@Autonomous(name = "BlueNearAuto", group = "Auto")
public class NearAuto  extends CommandOpMode {
    public Follower follower;
    public BucketRobot robot;
    public List<LynxModule> hubs;



    //Positions
    public Pose startingPos;
    public Pose patternPos1;
    public Pose patternPos2;
    public Pose patternPos3;
    public Pose shootingBackPos;
    public Pose shootingFrontPos;
    public Pose targetPos;

    //Paths


    public NearAuto(boolean isBlueAlliance) {
        BucketRobot.blueAlliance = isBlueAlliance;
    }
    public NearAuto() {
        this(true);
    }

    @Override
    public void initialize() {
        telemetry = new JoinedTelemetry(telemetry, PanelsTelemetry.INSTANCE.getFtcTelemetry());
        super.reset();

        //setup Positions
        startingPos = BucketRobot.createPose(56,144-8.5,Math.toRadians(270));
        targetPos = BucketRobot.createPose(8,136);



        // Initialize follower
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPos);
        robot = new BucketRobot(hardwareMap, telemetry, follower);
        follower.update();

        hubs = hardwareMap.getAll(LynxModule.class);
        hubs.forEach(hub -> hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL));


        //create Paths


        // Create auto sequence
        Command auto = new SequentialCommandGroup(

        );
        auto.schedule();
    }

    @Override
    public void run() {
        hubs.forEach(LynxModule::clearBulkCache);
        super.run();
        robot.run();

        telemetry.update();
    }
}
