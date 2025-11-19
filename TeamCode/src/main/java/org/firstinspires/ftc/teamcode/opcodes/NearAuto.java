package org.firstinspires.ftc.teamcode.opcodes;

import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.auto.AutoPoints;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.BucketRobot;

import java.util.List;

@Autonomous(name = "BlueNearAuto", group = "Auto")
public class NearAuto  extends CommandOpMode {
    public Follower follower;
    public BucketRobot robot;



    //Positions
    public Pose startingPos;
    public Pose patternPos1;
    public Pose patternPos2;
    public Pose patternPos3;
    public Pose shootingFarPos;
    public Pose shootingNearPos;
    public Pose targetPos;
    public Pose endingPos;

    //Paths
    public PathChain ShootLoaded;
    public PathChain CollectPattern1;
    public PathChain CollectPattern2;
    public PathChain CollectPattern3;
    public PathChain ShootCollected1;
    public PathChain ShootCollected2;
    public PathChain ShootCollected3;
    public PathChain MovetoEnd;

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
        startingPos = BucketRobot.createPose(AutoPoints.startingNearPos);
        targetPos = BucketRobot.createPose(AutoPoints.targetPos);

        patternPos1 = BucketRobot.createPose(AutoPoints.patternPos1);
        patternPos2 = BucketRobot.createPose(AutoPoints.patternPos2);
        patternPos3 = BucketRobot.createPose(AutoPoints.patternPos3);

        shootingFarPos = BucketRobot.createPose(AutoPoints.shootingFarPos);
        shootingNearPos = BucketRobot.createPose(AutoPoints.shootingNearPos);
        endingPos = BucketRobot.createPose(AutoPoints.endingNearPos);


        // Initialize follower
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPos);
        robot = new BucketRobot(hardwareMap, telemetry, follower);
        follower.update();




        //create Paths


        // Create auto sequence
        Command auto = new SequentialCommandGroup(

        );
        auto.schedule();
    }

    @Override
    public void run() {
        super.run();
        robot.run();

        telemetry.update();
    }
}
