package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;


import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.HashMap;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.subsystems.SubsystemGroup;


@Configurable
public class BucketRobot extends SubsystemGroup {
    public static final BucketRobot INSTANCE = new BucketRobot();

    public enum ARTIFACTPATTERN {
        NONE(0),
        GPP(21),
        PGP(22),
        PPG(23);
        private final int pattern;

        private ARTIFACTPATTERN(int pattern) {
            this.pattern = pattern;
        }

        public int getPattern() {
            return pattern;
        }
    }

    ;

    public ARTIFACTPATTERN pattern;

    public BucketRobot() {
        super(Outake.INSTANCE, Intake.INSTANCE, Camera.INSTANCE);
        pattern = ARTIFACTPATTERN.NONE;
    }

    private Command shootGPP = new SequentialGroup(
            Outake.INSTANCE.shootGreen,
            Outake.INSTANCE.shootPurple,
            Outake.INSTANCE.shootLoaded
    );
    private Command shootPGP = new SequentialGroup(
            Outake.INSTANCE.shootPurple,
            Outake.INSTANCE.shootGreen,
            Outake.INSTANCE.shootLoaded
    );

    private Command shootPPG = new SequentialGroup(
            Outake.INSTANCE.shootPurple,
            Outake.INSTANCE.shootPurple,
            Outake.INSTANCE.shootLoaded
    );
/*
    public Command shootPattern = new SwitchCommand<>(() -> BucketRobot.INSTANCE.pattern)
            .withCase(BucketRobot.ARTIFACTPATTERN.GPP, BucketRobot.INSTANCE.shootGPP)
            .withCase(BucketRobot.ARTIFACTPATTERN.PGP, BucketRobot.INSTANCE.shootPGP)
            .withCase(BucketRobot.ARTIFACTPATTERN.PPG, BucketRobot.INSTANCE.shootPPG);
*/
    @Override
    public void periodic() {
        if (!Camera.INSTANCE.cameraDisabled && pattern == ARTIFACTPATTERN.NONE) {
            for (AprilTagDetection detection : Camera.INSTANCE.currentDetections) {
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
}
