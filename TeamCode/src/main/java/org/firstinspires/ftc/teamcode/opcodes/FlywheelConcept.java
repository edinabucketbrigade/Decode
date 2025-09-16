package org.firstinspires.ftc.teamcode.opcodes;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import dev.nextftc.bindings.Bindings;
import dev.nextftc.bindings.Button;
import dev.nextftc.core.commands.CommandManager;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.extensions.pedro.PedroDriverControlled;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;
import dev.nextftc.hardware.controllable.MotorGroup;
import dev.nextftc.hardware.driving.DriverControlledCommand;
import dev.nextftc.hardware.impl.MotorEx;

@Configurable
@TeleOp(name = "Flywheel concept", group = "TeleOp")
public class FlywheelConcept extends NextFTCOpMode {

    Button gamepad1a;
    MotorEx motor1;
    MotorEx motor2;

    MotorGroup myMotorGroup;

    public static double motorSpeed = 0.75;

    @Override public void onInit() {
        addComponents(
                BindingsComponent.INSTANCE,
                BulkReadComponent.INSTANCE,
                CommandManager.INSTANCE);

        gamepad1a = Bindings.button(() -> gamepad1.a)
                .whenTrue(() -> myMotorGroup.setPower(motorSpeed))
                .whenFalse(() -> myMotorGroup.setPower(0))
        ;


        motor1 = new MotorEx("leftFrontDrive");
        motor2 = new MotorEx("rightFrontDrive");
        myMotorGroup = new MotorGroup( motor1, motor2.reversed());


    }
    @Override public void onWaitForStart() { }
    @Override public void onStartButtonPressed() { }
    @Override public void onUpdate() {

    }
    @Override public void onStop() { }

}
