package frc.robot.subsystem.drive;

import org.littletonrobotics.junction.AutoLogOutput;

import com.reduxrobotics.sensors.canandgyro.Canandgyro;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.DriveConstants;

public class SwerveDrive extends SubsystemBase {
    private final Canandgyro m_imu = new Canandgyro(DriveConstants.IMU_CAN_ID);
    
    @AutoLogOutput(key = "SwerveDrive/measuredAngle")
    public Rotation2d getMeasuredAngle() {
        return m_imu.getRotation2d();
    }

    @Override
    public void periodic() {
        this.getMeasuredAngle();
    }
     
}
