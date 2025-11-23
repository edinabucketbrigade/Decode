package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.DeferredCommand;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelRaceGroup;
import com.seattlesolvers.solverslib.command.Robot;
import com.seattlesolvers.solverslib.command.SelectCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.command.WaitUntilCommand;
import com.seattlesolvers.solverslib.pedroCommand.TurnCommand;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.auto.AutoPoints;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.HashMap;


@Configurable
public class BucketRobot extends Robot {
    private Intake intake;
    private Outake outake;
    private Camera camera;

    public static boolean blueAlliance;

    //store latest robot position in auto to use in tele
    public static Pose currentPos = null;

    public boolean fixedSpeed = false;
    public static double farSpeed = 0.85;
    public static double midSpeed = 0.7;
    public static double nearSpeed = 0.5;

    public enum ARTIFACTPATTERN {
        NONE(0),
        GPP(21),
        PGP(22),
        PPG(23);
        private final int pattern;

        ARTIFACTPATTERN(int pattern) {
            this.pattern = pattern;
        }

        public int getPattern() {
            return pattern;
        }
    }

    static ARTIFACTPATTERN pattern = ARTIFACTPATTERN.NONE;

    private static boolean disableCamera = false;
    private Telemetry telemetry;
    private Follower follower;
    private Pose targetPos;
    private double targetAngle = 0.0;

    public static boolean turnToTarget = true;

    public BucketRobot(HardwareMap hMap, Telemetry t, Follower f) {
        telemetry = t;
        follower = f;
        CommandScheduler.getInstance().setBulkReading(hMap, LynxModule.BulkCachingMode.MANUAL);
        targetPos = BucketRobot.createPose(AutoPoints.targetPos);


        outake = new Outake(hMap, t);
        intake = new Intake(hMap, t);
        if (!disableCamera) {
            camera = new Camera(hMap, t);
            register(outake, intake, camera);
        } else
            register(outake, intake);

        pattern = ARTIFACTPATTERN.NONE;


    }

    public Command enableIntake() {
        return new InstantCommand(() -> intake.StartIntake());
    }

    public Command disableIntake() {
        return new InstantCommand(() -> intake.StopIntake());
    }

    public Command enableOutake() {
        return new InstantCommand(() -> outake.StartOutake());
    }

    public Command disableOutake() {
        return new InstantCommand(() -> outake.StopOutake());
    }

    public Command shootRight() {
        return outake.shootR();
    }

    public Command shootLeft() {
        return outake.shootL();
    }

    public Command shootGreen() {
        return outake.shootGreen();
    }

    public Command shootPurple() {
        return outake.shootPurple();
    }

    public Command shootLoaded() {
        return outake.shootLoaded();
    }
    public Command turnToTargetAndShoot(Command command) {
        if (turnToTarget && targetAngle != 0.0)
        return new SequentialCommandGroup(
                new DeferredCommand(() -> new TurnCommand(follower, targetAngle,false),
                        null),
                command
        );
        else return command;
    }

    private Command shootGPP() {
        return new SequentialCommandGroup(
                shootGreen(),
                shootPurple(),
                new ParallelRaceGroup(
                        new WaitUntilCommand(outake::isLoaded),
                        new WaitCommand(2000)
                ),
                shootLoaded()
        );
    }

    private Command shootPGP() {
        return new SequentialCommandGroup(
                shootPurple(),
                shootGreen(),
                new ParallelRaceGroup(
                        new WaitUntilCommand(outake::isLoaded),
                        new WaitCommand(2000)
                ),
                shootLoaded()
        );
    }

    private Command shootPPG() {
        return new SequentialCommandGroup(
                shootPurple(),
                shootPurple(),
                new ParallelRaceGroup(
                        new WaitUntilCommand(outake::isLoaded),
                        new WaitCommand(2000)
                ),
                shootLoaded()
        );
    }

    public Command shootPattern() {
        return new SelectCommand(
                new HashMap<Object, Command>() {{
                    put(ARTIFACTPATTERN.GPP, shootGPP());
                    put(ARTIFACTPATTERN.PGP, shootPGP());
                    put(ARTIFACTPATTERN.PPG, shootPPG());
                    put(ARTIFACTPATTERN.NONE, shootPGP());
                }},
                () -> pattern);
    }

    public Command startAndShootPattern() {
        return new SequentialCommandGroup(
                enableOutake(),
                shootPattern(),
                disableOutake()
        );
    }

    private void setFlywheelSpeed() {
        if (fixedSpeed)
            Outake.speed = midSpeed;
        else {
            //Outake.speed = currentPos.distanceFrom(targetPos) / 153 * .3 + .55;
            double currentY = currentPos.getY();
            if (currentY >= 100)
                Outake.speed = nearSpeed;
            else if (currentY > 48)
                Outake.speed = midSpeed;
            else
                Outake.speed = farSpeed;
        }
    }

    @Override
    public void run() {
        currentPos = follower.getPose();

        telemetry.addData("Pose", "<%f,%f>:%f",
                currentPos.getX(),
                currentPos.getY(),
                Math.toDegrees(currentPos.getHeading()));


        //adjust power to hit goal
        setFlywheelSpeed();


        if (camera != null) {
            targetAngle = 0.0;
            for (AprilTagDetection detection : camera.currentDetections) {
                if (pattern == ARTIFACTPATTERN.NONE) {

                    if (detection.id == ARTIFACTPATTERN.GPP.getPattern()) {
                        pattern = ARTIFACTPATTERN.GPP;
                        break;
                    } else if (detection.id == ARTIFACTPATTERN.PGP.getPattern()) {
                        pattern = ARTIFACTPATTERN.PGP;
                        break;
                    } else if (detection.id == ARTIFACTPATTERN.PPG.getPattern()) {
                        pattern = ARTIFACTPATTERN.PPG;
                        break;
                    }
                }
                if (blueAlliance && detection.id == 20)
                    targetAngle = detection.ftcPose.bearing;
                else if (detection.id == 24)
                    targetAngle = detection.ftcPose.bearing;
            }
        }
        telemetry.addData("Pattern", "%s targeted at <%f,%f> distance %f",
                pattern.name(),
                targetPos.getX(),
                targetPos.getY(),
                currentPos.distanceFrom(targetPos) / 153

        );

        follower.update();
        super.run();
    }

    public static Pose createPose(double x, double y, double heading) {
        if (blueAlliance)
            return new Pose(x, y, heading);
        else
            return new Pose(x, y, heading).mirror();
    }

    public static Pose createPose(double x, double y) {
        if (blueAlliance)
            return new Pose(x, y);
        else
            return new Pose(x, y).mirror();
    }

    public static Pose createPose(double[] pos) {
        if (pos.length == 2)
            return createPose(pos[0], pos[1]);
        if (pos.length == 3)
            return createPose(pos[0], pos[1], Math.toRadians(pos[2]));
        return null;
    }

}
