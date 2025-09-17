package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.Encoder;
import com.pedropathing.ftc.localization.constants.DriveEncoderConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Constants {
    public static double ForwardZeroPowerAcceleration = 1.0;
    public static double LateralZeroPowerAcceleration = 1.0;

    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(5)
            //.forwardZeroPowerAcceleration(ForwardZeroPowerAcceleration)
            //.lateralZeroPowerAcceleration(LateralZeroPowerAcceleration)
            ;

    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

    public static double XVelocity = 1.0;
    public static double YVelocity = 1.0;

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("rightFrontDrive")
            .rightRearMotorName("rightBackDrive")
            .leftRearMotorName("leftBackDrive")
            .leftFrontMotorName("leftFrontDrive")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            //.xVelocity(XVelocity)
            //.yVelocity(YVelocity)
            ;
    public static double RobotWidth =15.5;
    public static double RobotLength =17.0;
    public static double ForwardTicksToInches = 0.005806641378857757;
    public static double StrafeTicksToInches = 0.006122924855949316;
    public static double TurnTicksToInches = 0.014282963033037463;

    public static DriveEncoderConstants localizerConstants = new DriveEncoderConstants()
            .rightFrontMotorName("rightFrontDrive")
            .rightRearMotorName("rightBackDrive")
            .leftRearMotorName("leftBackDrive")
            .leftFrontMotorName("leftFrontDrive")
            .leftFrontEncoderDirection(Encoder.REVERSE)
            .leftRearEncoderDirection(Encoder.REVERSE)
            .rightFrontEncoderDirection(Encoder.FORWARD)
            .rightRearEncoderDirection(Encoder.FORWARD)
            .robotWidth(RobotWidth)
            .robotLength(RobotLength)
            .forwardTicksToInches(ForwardTicksToInches)
            .strafeTicksToInches(StrafeTicksToInches)
            .turnTicksToInches(TurnTicksToInches)
            ;

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .driveEncoderLocalizer(localizerConstants)
                .build();
    }
}
