package org.firstinspires.ftc.teamcode.opcodes;



import static dev.nextftc.bindings.Bindings.*;

import com.bylazar.telemetry.PanelsTelemetry;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.bylazar.telemetry.JoinedTelemetry;


import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.BucketRobot;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Outake;


import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.extensions.pedro.PedroDriverControlled;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;
import dev.nextftc.hardware.driving.DriverControlledCommand;

@TeleOp(name = "MainTeleOp", group = "TeleOp")
public class MainTeleOp extends NextFTCOpMode {
    public MainTeleOp() {
        addComponents(
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE,
                new SubsystemComponent(BucketRobot.INSTANCE),
                new PedroComponent(Constants::createFollower)
        );
    }
    @Override public void onWaitForStart() { }
    @Override public void onStartButtonPressed() {
        DriverControlledCommand driverControlled = new PedroDriverControlled(
                Gamepads.gamepad1().leftStickY(),
                Gamepads.gamepad1().leftStickX(),
                Gamepads.gamepad1().rightStickX()
        );
        driverControlled.schedule();
    }
    @Override public void onStop() { }
    @Override public void onInit() {
        telemetry = new JoinedTelemetry(telemetry, PanelsTelemetry.INSTANCE.getFtcTelemetry());


        Gamepads.gamepad1().leftTrigger().greaterThan(0.2)
                .whenBecomesTrue(Outake.INSTANCE.shootL);
        Gamepads.gamepad1().rightTrigger().greaterThan(0.2)
                .whenBecomesTrue(Outake.INSTANCE.shootR);

        Gamepads.gamepad1().leftBumper()
                .toggleOnBecomesTrue()
                .whenBecomesTrue(Intake.INSTANCE.on)
                .whenBecomesFalse(Intake.INSTANCE.off);

        Gamepads.gamepad1().rightBumper()
                .toggleOnBecomesTrue()
                .whenBecomesTrue(Outake.INSTANCE.on)
                .whenBecomesFalse(Outake.INSTANCE.off);
        // A shoots the green ball
        Gamepads.gamepad1().a()
                .whenBecomesTrue(Outake.INSTANCE.shootGreen);
        // B shoots the purple ball
        Gamepads.gamepad1().b()
                .whenBecomesTrue(Outake.INSTANCE.shootPurple);
        // X shoots the collum with a ball in it (shoots loaded)
        Gamepads.gamepad1().x()
                .whenBecomesTrue(Outake.INSTANCE.shootLoaded);
    }

    @Override public void onUpdate() {

        telemetry.addData("Pose", "<%d,%d>:%d",
                PedroComponent.follower().getPose().getX(),
                PedroComponent.follower().getPose().getY(),
                PedroComponent.follower().getPose().getHeading());
        telemetry.update();
    }
}