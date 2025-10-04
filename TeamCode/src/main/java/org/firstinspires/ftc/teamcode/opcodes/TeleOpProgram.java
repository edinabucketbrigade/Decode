package org.firstinspires.ftc.teamcode.opcodes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.extensions.pedro.PedroDriverControlled;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;
import dev.nextftc.hardware.driving.DriverControlledCommand;

@TeleOp(name="Test Name", group="Linear OpMode")

    public class TeleOpProgram extends NextFTCOpMode {
        public TeleOpProgram() {
            addComponents(
                    BulkReadComponent.INSTANCE,
                    BindingsComponent.INSTANCE,
                    new PedroComponent(Constants::createFollower)
            );
        }
    @Override public void onInit() {
        DriverControlledCommand driverControlled = new PedroDriverControlled(
            Gamepads.gamepad1().leftStickY(),
            Gamepads.gamepad1().leftStickX(),
            Gamepads.gamepad1().rightStickX(),
            true
        );
        driverControlled.schedule();}
    }
