// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    //Constants for the drive train
    public static class Drivetrain{
        //Universal motor speed limit
        public static final double speedLmt = 1;

        //Drive train encoder values
        public static final double gearboxRatio = 1;
        public static final double inchesPerRotation = 1;
    }

    //Motor ports
    public static final int 
        FRONT_ONE_PIN = 1,
        FRONT_TWO_PIN = 2,
        BACK_ONE_PIN = 3,
        BACK_TWO_PIN = 4;

    public static final int joy1 = 0, joy2 = 1, joy3 = 2;


}
