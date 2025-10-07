package org.firstinspires.ftc.teamcode.opcodes;


import com.pedropathing.follower.Follower;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp(name = "MainTeleOp", group = "TeleOp")
public class MainTeleOp extends CommandOpMode {
    Follower follower;

    @Override
    public void initialize() {
        follower = Constants.createFollower(hardwareMap);
        super.reset();
        follower.update();

        follower.startTeleopDrive();
    }

    @Override
    public void run() {
        super.run();
        follower.update();

        follower.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, true);

    }
}