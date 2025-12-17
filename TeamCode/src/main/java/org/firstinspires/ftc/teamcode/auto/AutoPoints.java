package org.firstinspires.ftc.teamcode.auto;

import com.bylazar.configurables.annotations.Configurable;

//make points configurable
@Configurable
public class AutoPoints {
    public static double collectSpeed = 1.0;
    public static double moveSpeed = 1.0;

    public static double[] startingFarPos = {56,8.5,90};
    public static double[] startingNearPos = {56,135.5,90};
    public static double[] patternPos1 = {23,83.5,180};
    public static double[] patternPos2 = {23,59,180};
    public static double[] patternPos3 = {23,35,180};
    public static double[] shootingFarPos = {56,11};
    public static double[] shootingNearPos = {50,84};
    public static double[] targetPos = {5,138};
    public static double[] endingFarPos = {36,13};
    public static double[] endingNearPos = {16,104};
}
