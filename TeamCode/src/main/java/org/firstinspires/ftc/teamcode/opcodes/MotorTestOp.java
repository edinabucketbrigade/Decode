package org.firstinspires.ftc.teamcode.opcodes;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.button.Trigger;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.BucketRobot;


import java.util.List;

@TeleOp(name = "MotorTestOp", group = "TeleOp")
public class MotorTestOp extends CommandOpMode {
    Follower follower;
    private TelemetryManager telemetryM;
    private GamepadEx toolOp;

    private BucketRobot robot;


    private List<LynxModule> hubs;
    private ElapsedTime runtime;

    @Override
    public void initialize() {
        //follower = Constants.createFollower(hardwareMap);
        super.reset();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        runtime = new ElapsedTime();

        robot = new BucketRobot(hardwareMap, telemetryM);

        // initialize Pedro
        follower.update();
        follower.startTeleopDrive();

        // here, we are setting the bulk caching mode to manual so all hardware reads
        // for the motors can be read in one hardware call.
        // we do this in order to decrease our loop time
        hubs = hardwareMap.getAll(LynxModule.class);
        hubs.forEach(hub -> hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL));

        toolOp = new GamepadEx(gamepad1);
/*        toolOp.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).
                whenPressed(robot.enableIntakeCommand()).
                whenReleased(robot.disableIntakeCommand());

        toolOp.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).
                whenPressed(robot.enableShooterCommand()).
                whenReleased(robot.disableShooterCommand());

        toolOp.getGamepadButton(GamepadKeys.Button.A).
                whenPressed(robot.shootLeftCommand());

        toolOp.getGamepadButton(GamepadKeys.Button.B).
                whenPressed(robot.shootRightCommand());
*/
        //setup base Telemetry

        telemetryM.update();
        //telemetryM.debug("position", follower.getPose());

    }

    @Override
    public void run() {
        super.run();

        telemetryM.update();
        follower.update();
        hubs.forEach(LynxModule::clearBulkCache);

        robot.run();

        if (toolOp.isDown(GamepadKeys.Button.LEFT_BUMPER)) robot.enableIntake();
        else robot.disableIntake();
        if (toolOp.isDown(GamepadKeys.Button.RIGHT_BUMPER)) robot.enableShooter();
        else robot.disableShooter();

        if (toolOp.isDown(GamepadKeys.Button.A)) robot.shootLeft();
        if (toolOp.isDown(GamepadKeys.Button.B)) robot.shootRight();

        if (toolOp.isDown(GamepadKeys.Button.X)) robot.setLeft();
        if (toolOp.isDown(GamepadKeys.Button.Y)) robot.setRight();

        follower.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, true);
        //telemetryM.debug("position", follower.getPose());
        telemetryM.debug("Elapsed time", runtime);

    }

}
