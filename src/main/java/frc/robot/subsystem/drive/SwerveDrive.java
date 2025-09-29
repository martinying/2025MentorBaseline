package frc.robot.subsystem.drive;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.reduxrobotics.sensors.canandgyro.Canandgyro;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.DriveConstants;
import frc.robot.subsystem.SwerveModule;

public class SwerveDrive extends SubsystemBase {
    private final Canandgyro m_imu = new Canandgyro(DriveConstants.IMU_CAN_ID);
    private final SwerveModule[] swerveModules = new SwerveModule[4];

    public SwerveDrive() {
        swerveModules[DriveConstants.FRONT_LEFT_MODULE_INDEX] = new SwerveModule(DriveConstants.FRONT_LEFT_KRAKEN_ID,DriveConstants.FRONT_LEFT_FALCON_ID,DriveConstants.FRONT_LEFT_ENCODER_ID);
        swerveModules[DriveConstants.FRONT_RIGHT_MODULE_INDEX] = new SwerveModule(DriveConstants.FRONT_RIGHT_KRAKEN_ID,DriveConstants.FRONT_RIGHT_FALCON_ID,DriveConstants.FRONT_RIGHT_ENCODER_ID);
        swerveModules[DriveConstants.BACK_LEFT_MODULE_INDEX] = new SwerveModule(DriveConstants.BACK_LEFT_KRAKEN_ID,DriveConstants.BACK_LEFT_FALCON_ID,DriveConstants.BACK_LEFT_ENCODER_ID);
        swerveModules[DriveConstants.BACK_RIGHT_MODULE_INDEX] = new SwerveModule(DriveConstants.BACK_RIGHT_KRAKEN_ID,DriveConstants.BACK_RIGHT_FALCON_ID,DriveConstants.BACK_RIGHT_ENCODER_ID);
    }
    
    @AutoLogOutput(key = "SwerveDrive/measuredAngle")
    public Rotation2d getMeasuredAngle() {
        return m_imu.getRotation2d();
    }

    @Override
    public void periodic() {
        this.getMeasuredAngle();
        for(int counter = 0; counter < 4; counter++) {
            swerveModules[counter].updateInputs();
            switch ( counter ) {
                case DriveConstants.FRONT_LEFT_MODULE_INDEX:
                    Logger.processInputs("SwerveDrive/ModuleInput/FrontLeft",swerveModules[counter].getInputs());
                    break;
                case DriveConstants.FRONT_RIGHT_MODULE_INDEX:
                    Logger.processInputs("SwerveDrive/ModuleInput/FrontRight",swerveModules[counter].getInputs());
                    break;
                case DriveConstants.BACK_LEFT_MODULE_INDEX:
                    Logger.processInputs("SwerveDrive/ModuleInput/BackLeft",swerveModules[counter].getInputs());
                    break;
                case DriveConstants.BACK_RIGHT_MODULE_INDEX:
                    Logger.processInputs("SwerveDrive/ModuleInput/BackRight",swerveModules[counter].getInputs());
                    break;
            }
        }
    }
     
    public void driveRobot(ChassisSpeeds chassisSpeed) {
        Logger.recordOutput("SwerveDrive/chassisSpeed", chassisSpeed);

        SwerveModuleState [] desiredSwerveModuleStates = DriveConstants.DRIVE_KINEMATICS.toSwerveModuleStates(chassisSpeed);
        Logger.recordOutput("SwerveDrive/DesiredSwerveModuleStates/FrontLeft", desiredSwerveModuleStates[DriveConstants.FRONT_LEFT_MODULE_INDEX]);
        Logger.recordOutput("SwerveDrive/DesiredSwerveModuleStates/FrontRight", desiredSwerveModuleStates[DriveConstants.FRONT_RIGHT_MODULE_INDEX]);
        Logger.recordOutput("SwerveDrive/DesiredSwerveModuleStates/BackLeft", desiredSwerveModuleStates[DriveConstants.BACK_LEFT_MODULE_INDEX]);
        Logger.recordOutput("SwerveDrive/DesiredSwerveModuleStates/BackRight", desiredSwerveModuleStates[DriveConstants.BACK_RIGHT_MODULE_INDEX]);

        for(int counter = 0; counter < 4; counter++) {
            swerveModules[counter].setModuleState(desiredSwerveModuleStates[counter],counter);
        }
        
    }
}
