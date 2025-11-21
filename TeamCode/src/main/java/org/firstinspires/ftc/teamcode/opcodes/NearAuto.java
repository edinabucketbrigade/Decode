package org.firstinspires.ftc.teamcode.opcodes;

import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;

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

        ShootLoaded = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(startingPos, shootingNearPos)
                )
                .setHeadingInterpolation(HeadingInterpolator.facingPoint(targetPos))
                .build();
        CollectPattern1 = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                shootingNearPos,
                                BucketRobot.createPose(shootingNearPos.getX(), patternPos1.getY()),
                                patternPos1
                        )
                )
                // from sample https://pedropathing.com/docs/pathing/examples/teleop
                .setHeadingInterpolation(
                        HeadingInterpolator.linearFromPoint(
                                follower::getHeading,
                                patternPos1.getHeading(),
                                0.8
                        )
                )
                .build();
        ShootCollected1 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(patternPos1, shootingNearPos)
                )
                .setHeadingInterpolation(HeadingInterpolator.facingPoint(targetPos))
                .build();
        CollectPattern2 = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                shootingNearPos,
                                BucketRobot.createPose(shootingFarPos.getX(), patternPos2.getY()),
                                patternPos2
                        )
                )
                // from sample https://pedropathing.com/docs/pathing/examples/teleop
                .setHeadingInterpolation(
                        HeadingInterpolator.linearFromPoint(
                                follower::getHeading,
                                patternPos2.getHeading(),
                                0.8
                        )
                )
                .build();
        ShootCollected2 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(patternPos2, shootingNearPos)
                )
                .setHeadingInterpolation(HeadingInterpolator.facingPoint(targetPos))
                .build();
        CollectPattern3 = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                shootingNearPos,
                                BucketRobot.createPose(shootingFarPos.getX(), patternPos3.getY()),
                                patternPos3
                        )
                )
                // from sample https://pedropathing.com/docs/pathing/examples/teleop
                .setHeadingInterpolation(
                        HeadingInterpolator.linearFromPoint(
                                follower::getHeading,
                                patternPos3.getHeading(),
                                0.8
                        )
                )
                .build();
        ShootCollected3 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(patternPos3, shootingFarPos)
                )
                .setHeadingInterpolation(HeadingInterpolator.facingPoint(targetPos))
                .build();
        MovetoEnd = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(shootingFarPos, endingPos)
                )
                .setTangentHeadingInterpolation()
                .build();

        // Create auto sequence
        Command auto = new SequentialCommandGroup(
                new FollowPathCommand(follower, ShootLoaded),
                robot.startAndShootPattern(),

                robot.enableIntake(),

                new FollowPathCommand(follower, CollectPattern1),
                new FollowPathCommand(follower, ShootCollected1),
                robot.startAndShootPattern(),

                new FollowPathCommand(follower, CollectPattern2),
                new FollowPathCommand(follower, ShootCollected2),
                robot.startAndShootPattern(),

                new FollowPathCommand(follower, CollectPattern3),
                new FollowPathCommand(follower, ShootCollected3),
                robot.startAndShootPattern(),

                robot.disableIntake(),
                new FollowPathCommand(follower, MovetoEnd)
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
