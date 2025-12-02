package org.firstinspires.ftc.teamcode.auto;

import com.bylazar.configurables.annotations.Configurable;

//make points configurable
@Configurable
public class AutoPoints {
    public static double collectSpeed = 1.0;

    public static double[] startingFarPos = {56,8.5,90};
    public static double[] startingNearPos = {56,135.5,90};
    public static double[] patternPos1 = {19,84,180};
    public static double[] patternPos2 = {19,60,180};
    public static double[] patternPos3 = {19,36,180};
    public static double[] shootingFarPos = {56,11};
    public static double[] shootingNearPos = {50,84};
    public static double[] targetPos = {7,137};
    public static double[] endingFarPos = {36,13};
    public static double[] endingNearPos = {36,13};
}
