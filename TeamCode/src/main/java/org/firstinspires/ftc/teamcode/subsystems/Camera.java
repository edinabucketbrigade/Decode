package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;

@Configurable
public class Camera implements Subsystem {
    public static final Camera INSTANCE = new Camera();
    /**
     * The variable to store our instance of the AprilTag processor.
     */
    private AprilTagProcessor aprilTag;

    /**
     * The variable to store our instance of the vision portal.
     */
    private VisionPortal visionPortal;

    public List<AprilTagDetection> currentDetections;
    public final boolean cameraDisabled = true;

    public Camera() {
        if (!cameraDisabled) {
            aprilTag = AprilTagProcessor.easyCreateWithDefaults();


            visionPortal = VisionPortal.easyCreateWithDefaults(
                    ActiveOpMode.hardwareMap().get(WebcamName.class, "Webcam"), aprilTag);
        }
    }

    @Override
    public void periodic() {
        if (!cameraDisabled) currentDetections = aprilTag.getDetections();
    }

}
