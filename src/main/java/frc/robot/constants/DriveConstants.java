package frc.robot.constants;

import edu.wpi.first.math.geometry.Translation2d;

import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

public class DriveConstants {

    public static final int IMU_CAN_ID = 0; //CAN

    public static final double MAX_SPEED_METER_PER_SECCONDS_DEFAULT_VALUE = 5;

    public static final double WHEEL_RADIUS_DEFAULT_VALUE = 2;

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

}
