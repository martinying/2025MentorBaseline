package frc.robot.constants;

import edu.wpi.first.math.geometry.Translation2d;

import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

public class DriveConstants {
    private DriveConstants() {
        //hide constructor to prevent instantiation
    }

    public static final int IMU_CAN_ID = 0; //CAN

    public static final double MAX_SPEED_METER_PER_SECONDS_DEFAULT_VALUE = 5;

    //GEAR RATIO IS PULLED FROM SITE BELOW
    //https://www.swervedrivespecialties.com/products/mk4i-swerve-module
    public static final double SWERVE_MODULE_DRIVE_MOTOR_GEAR_RATIO = 6.25;
    
    public static final double WHEEL_RADIUS_DEFAULT_VALUE = 2;

    public static final double WHEEL_DIAMETER_IN_METERS = Units.inchesToMeters(4);

    public static final SwerveDriveKinematics DRIVE_KINEMATICS = new SwerveDriveKinematics(
        new Translation2d(0.3556 - 0.065, 0.3556 - 0.068), //front left
        new Translation2d(0.3556 - 0.066, -0.3556 + 0.066), //front right
        new Translation2d(0.3556 - 0.645, -0.3556 + 0.644), //back left
        new Translation2d(0.3556 - 0.644, -0.3556 + 0.063) //back right
    );

    public static final int FRONT_LEFT_MODULE_INDEX = 0;
    public static final int FRONT_RIGHT_MODULE_INDEX = 1;
    public static final int BACK_LEFT_MODULE_INDEX = 2;
    public static final int BACK_RIGHT_MODULE_INDEX = 3;

    public static final int FRONT_RIGHT_KRAKEN_ID = 5;
    public static final int FRONT_RIGHT_FALCON_ID = 14;
    public static final int FRONT_LEFT_KRAKEN_ID = 1;
    public static final int FRONT_LEFT_FALCON_ID = 11;
    public static final int BACK_RIGHT_KRAKEN_ID = 3;
    public static final int BACK_RIGHT_FALCON_ID = 13;
    public static final int BACK_LEFT_KRAKEN_ID = 2;
    public static final int BACK_LEFT_FALCON_ID = 12;

    public static final int FRONT_RIGHT_ENCODER_ID = 3;
    public static final int FRONT_LEFT_ENCODER_ID = 2;
    public static final int BACK_RIGHT_ENCODER_ID = 4;
    public static final int BACK_LEFT_ENCODER_ID = 0;

    public static final double FRONT_RIGHT_ABSOLUTE_ENCODER_OFFSET = 0.19305;
    public static final double FRONT_LEFT_ABSOLUTE_ENCODER_OFFSET = 0.86125;
    public static final double BACK_RIGHT_ABSOLUTE_ENCODER_OFFSET = 0.6341;
    public static final double BACK_LEFT_ABSOLUTE_ENCODER_OFFSET = 0.0335;
}
