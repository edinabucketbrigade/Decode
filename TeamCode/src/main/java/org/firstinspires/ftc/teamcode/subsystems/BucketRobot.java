package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.MathFunctions;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelRaceGroup;
import com.seattlesolvers.solverslib.command.Robot;
import com.seattlesolvers.solverslib.command.SelectCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.command.WaitUntilCommand;
import com.seattlesolvers.solverslib.pedroCommand.TurnToCommand;
import com.seattlesolvers.solverslib.util.InterpLUT;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.auto.AutoPoints;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.HashMap;


@Configurable
public class BucketRobot extends Robot {
    private final Intake intake;
    private final Outake outake;
    private final Camera camera;

    public static boolean blueAlliance;

    //store latest robot position in auto to use in tele
    public static Pose currentPos = null;

    public boolean fixedSpeed = false;
    public static double farSpeed = 0.88;
    public static double midSpeed = 0.68;
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

    private final Telemetry telemetry;
    private final Follower follower;
    private final Pose targetPos;

    public boolean turnToTarget = false;

    private final InterpLUT lut;

    public BucketRobot(HardwareMap hMap, Telemetry t, Follower f) {
        telemetry = t;
        follower = f;
        CommandScheduler.getInstance().setBulkReading(hMap, LynxModule.BulkCachingMode.MANUAL);
        targetPos = BucketRobot.createPose(AutoPoints.targetPos);


        outake = new Outake(hMap, t);
        intake = new Intake(hMap, t);

        camera = new Camera(hMap, t);
        register(outake, intake, camera);

        pattern = ARTIFACTPATTERN.NONE;

        // interpolated flywheel velocity lookuptable by distance
        lut = new InterpLUT();

//Adding each val with a key
        lut.add(0, nearSpeed);
        lut.add(51.7,.55);
        lut.add(71,.615);
        lut.add(102,.7);
        lut.add(140,.85);
        lut.add(200, 1.0);

//generating final equation
        lut.createLUT();
    }

    public Command enableIntake() {
        return new InstantCommand(intake::StartIntake);
    }

    public Command disableIntake() {
        return new InstantCommand(intake::StopIntake);
    }

    public Command enableOutake() {
        return new InstantCommand(outake::StartOutake);
    }

    public Command disableOutake() {
        return new InstantCommand(outake::StopOutake);
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

    public Command shootBoth() {
        return outake.shootBoth();
    }

    private Command shootGPP() {
        return new SequentialCommandGroup(
                shootGreen(),
                shootPurple(),
                new ParallelRaceGroup(
                        new WaitUntilCommand(outake::isLoaded),
                        new WaitCommand(2000)
                ),
                shootBoth()
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
                shootBoth()
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
                shootBoth()
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

            double dist = currentPos.distanceFrom(targetPos);
            Outake.speed = lut.get(dist);

/*            if (dist >= 130)
                Outake.speed = farSpeed;
            else if (dist > 60)
                Outake.speed = midSpeed;
            else
                Outake.speed = nearSpeed;
*/

        }
    }

    @Override
    public void run() {
        currentPos = follower.getPose();

        double targetAngle = MathFunctions.normalizeAngle(Math.atan2(
                targetPos.getY() - currentPos.getY(),
                targetPos.getX() - currentPos.getX()));

        telemetry.addData("Pose", "<%f,%f>:%f distance %f at %f degrees",
                currentPos.getX(),
                currentPos.getY(),
                Math.toDegrees(currentPos.getHeading()),
                currentPos.distanceFrom(targetPos),
                Math.toDegrees(targetAngle)
        );


        //adjust power to hit goal
        setFlywheelSpeed();


        if (camera != null) {
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
            }
        }

        if (turnToTarget)
            new TurnToCommand(follower, targetAngle).schedule();


        telemetry.addData("Pattern",
                pattern.name()
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
