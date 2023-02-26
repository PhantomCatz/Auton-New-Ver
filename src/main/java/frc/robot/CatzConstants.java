package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;

public class CatzConstants {

    /*
     * This is the distance of each wheels from the robot's center. 
     * You multiply the length in inches by 0.0254
     * Forward is positive x and left is positive y
     */
    public static final Translation2d SWERVE_LEFT_FRONT_LOCATION = new Translation2d(0.30155700054494, 0.30155700054494);
    public static final Translation2d SWERVE_LEFT_BACK_LOCATION = new Translation2d(0.30155700054494, -0.30155700054494);
    public static final Translation2d SWERVE_RIGHT_FRONT_LOCATION = new Translation2d(-0.30155700054494, 0.30155700054494);
    public static final Translation2d SWERVE_RIGHT_BACK_LOCATION = new Translation2d(-0.30155700054494, -0.30155700054494);

    public static final double MAX_AUTON_SPEED_METERS_PER_SECOND = 2;

    static final double GEAR_RATIO = 1.0/6.75;

    static final double TALONFX_INTEGRATED_ENC_CNTS_PER_REV      = 2048.0;
    static final double DRVTRAIN_WHEEL_RADIUS                    = 2.0;
    static final double DRVTRAIN_WHEEL_CIRCUMFERENCE             = (2.0 * Math.PI * DRVTRAIN_WHEEL_RADIUS);
    public static final double DRVTRAIN_ENC_COUNTS_TO_INCH       = GEAR_RATIO * DRVTRAIN_WHEEL_CIRCUMFERENCE / TALONFX_INTEGRATED_ENC_CNTS_PER_REV;
}
