package org.firstinspires.ftc.teamcode.subsystems;

import android.hardware.HardwareBuffer;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@Configurable
public class Camera extends SubsystemBase  {
    private Telemetry telemetryM;
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
    };

    public ARTIFACTPATTERN pattern;
    /**
     * The variable to store our instance of the AprilTag processor.
     */
    private AprilTagProcessor aprilTag;

    /**
     * The variable to store our instance of the vision portal.
     */
    private VisionPortal visionPortal;
    public Camera(HardwareMap hMap, Telemetry m) {
        telemetryM = m;

        aprilTag = AprilTagProcessor.easyCreateWithDefaults();

        visionPortal = VisionPortal.easyCreateWithDefaults(
                hMap.get(WebcamName.class, "Webcam"), aprilTag);
        pattern = ARTIFACTPATTERN.NONE;
    }

    @Override
    public void periodic() {
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();

        for (AprilTagDetection detection : currentDetections) {
            if (detection.id == ARTIFACTPATTERN.GPP.getPattern()) {
                pattern = ARTIFACTPATTERN.GPP;
            } else if (detection.id == ARTIFACTPATTERN.PGP.getPattern()) {
                pattern = ARTIFACTPATTERN.PGP;
            } else if (detection.id == ARTIFACTPATTERN.PPG.getPattern()) {
                pattern = ARTIFACTPATTERN.PPG;
            }
        }
    }

    public ARTIFACTPATTERN getPattern() {return pattern;}
}
