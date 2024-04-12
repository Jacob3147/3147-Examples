package frc.robot.Utility;

import edu.wpi.first.math.util.Units;

public class Constants 
{
    public static final class DriveConstants
    {
        public static int kFrontLeftCANID = 1;
        public static int kBackLeftCANID = 2;
        public static int kFrontRightCANID = 3;
        public static int kBackRightCANID = 4;

        public static double kWheelWidth = Units.inchesToMeters(20);
        public static double kMetersPerRevolution = Units.inchesToMeters(4)*Math.PI / 8.45;
        public static double kMPS_Per_RPM = kMetersPerRevolution * 60;
        public static double maxSpeed = 3;


        public static double turnNerf = 0.7;
        public static double turnDeadband = 0.08;
        public static double fwdDeadband = 0.08;
        public static double maxTurnSpeedRadiansPerSecond = 3;
        public static double maxSpeedMetersPerSecond = 3.5;
    }
}
