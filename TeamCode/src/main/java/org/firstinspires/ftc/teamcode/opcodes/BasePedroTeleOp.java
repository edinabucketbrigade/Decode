package org.firstinspires.ftc.teamcode.opcodes;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp(name = "BasePedroTeleOp", group = "TeleOp")
public class BasePedroTeleOp extends OpMode {
    private Follower follower;
    private TelemetryManager telemetryM;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.update();

        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        telemetryM.update();
        telemetryM.debug("position", follower.getPose());
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
    }

    @Override
    public void loop() {
        telemetryM.update();
        follower.update();
        follower.setTeleOpDrive(-gamepad1.left_stick_y, gamepad1.right_stick_x, -gamepad1.left_stick_x, true);
        telemetryM.debug("position", follower.getPose());
    }
}
