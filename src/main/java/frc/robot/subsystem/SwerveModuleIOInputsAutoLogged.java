package frc.robot.subsystem;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystem.SwerveModule.SwerveModuleIOInputs;

public class SwerveModuleIOInputsAutoLogged extends SwerveModuleIOInputs implements LoggableInputs {

    @Override
    public void toLog(LogTable table) {
        table.put("driveVelocityRadPerSec", driveVelocityRadPerSec);
        table.put("drivePositionRad", drivePositionRad);
        table.put("driveAngularAcceleration", driveAngularAcceleration);
        
        table.put("turnMotorControllerPosition", turnMotorControllerPosition);
        table.put("absoluteEncoderPosition", absoluteEncoderPosition);
        table.put("turnMotorVelocityRadPerSec", turnMotorVelocityRadPerSec);
        table.put("turnMotorAngularAcceleration", turnMotorAngularAcceleration);        
    }

    @Override
    public void fromLog(LogTable table) {
        driveVelocityRadPerSec = table.get("driveVelocityRadPerSec", driveVelocityRadPerSec);
        drivePositionRad = table.get("drivePositionRad", drivePositionRad);
        driveAngularAcceleration = table.get("driveAngularAcceleration", driveAngularAcceleration);    

        turnMotorControllerPosition = table.get("turnMotorControllerPosition", turnMotorControllerPosition);
        absoluteEncoderPosition = table.get("absoluteEncoderPosition", absoluteEncoderPosition);
        turnMotorVelocityRadPerSec = table.get("turnMotorVelocityRadPerSec", turnMotorVelocityRadPerSec);    
        turnMotorAngularAcceleration = table.get("turnMotorAngularAcceleration", turnMotorAngularAcceleration);    
    }

}
