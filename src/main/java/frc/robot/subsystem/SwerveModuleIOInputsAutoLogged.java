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
        table.put("driveBridgeOutput", driveBridgeOutput);
        table.put("driveControlSystemTotalOutput", driveControlSystemTotalOutput);
        table.put("driveControlSystemProportionalOutput", driveControlSystemProportionalOutput);
        table.put("driveControlSystemReference", driveControlSystemReference);
        table.put("driveDutyCycle", driveDutyCycle);
        table.put("driveVoltage", driveVoltage);
        table.put("driveSupplyCurrent", driveSupplyCurrent);
        table.put("driveSupplyVoltage", driveSupplyVoltage);
        table.put("driveTorqueCurrent", driveTorqueCurrent);
        
        table.put("turnMotorControllerPosition", turnMotorControllerPosition);
        table.put("absoluteEncoderPosition", absoluteEncoderPosition);
        table.put("turnMotorVelocityRadPerSec", turnMotorVelocityRadPerSec);
        table.put("turnMotorAngularAcceleration", turnMotorAngularAcceleration);
        table.put("turnBridgeOutput", turnBridgeOutput);
        table.put("turnDutyCycle", turnDutyCycle);
        table.put("turnVoltage", turnVoltage);
        table.put("turnSupplyCurrent", turnSupplyCurrent);
        table.put("turnSupplyVoltage", turnSupplyVoltage);
        table.put("turnTorqueCurrent", turnTorqueCurrent);
    }

    @Override
    public void fromLog(LogTable table) {
        driveVelocityRadPerSec = table.get("driveVelocityRadPerSec", driveVelocityRadPerSec);
        drivePositionRad = table.get("drivePositionRad", drivePositionRad);
        driveAngularAcceleration = table.get("driveAngularAcceleration", driveAngularAcceleration);
        driveBridgeOutput = table.get("driveBridgeOutput", driveBridgeOutput);
        driveControlSystemTotalOutput = table.get("driveControlSystemTotalOutput", driveControlSystemTotalOutput);
        driveControlSystemProportionalOutput = table.get("driveControlSystemProportionalOutput", driveControlSystemProportionalOutput);
        driveControlSystemReference = table.get("driveControlSystemReference", driveControlSystemReference);
        driveDutyCycle = table.get("driveDutyCycle", driveDutyCycle);
        driveVoltage = table.get("driveVoltage", driveVoltage);
        driveSupplyCurrent = table.get("driveSupplyCurrent", driveSupplyCurrent);
        driveSupplyVoltage = table.get("driveSupplyVoltage", driveSupplyVoltage);
        driveTorqueCurrent = table.get("driveTorqueCurrent", driveTorqueCurrent);

        turnMotorControllerPosition = table.get("turnMotorControllerPosition", turnMotorControllerPosition);
        absoluteEncoderPosition = table.get("absoluteEncoderPosition", absoluteEncoderPosition);
        turnMotorVelocityRadPerSec = table.get("turnMotorVelocityRadPerSec", turnMotorVelocityRadPerSec);    
        turnMotorAngularAcceleration = table.get("turnMotorAngularAcceleration", turnMotorAngularAcceleration);
        turnBridgeOutput = table.get("turnBridgeOutput", turnBridgeOutput);
        turnDutyCycle = table.get("turnDutyCycle", turnDutyCycle);
        turnVoltage = table.get("turnVoltage", turnVoltage);
        turnSupplyCurrent = table.get("turnSupplyCurrent", turnSupplyCurrent);
        turnSupplyVoltage = table.get("turnSupplyVoltage", turnSupplyVoltage);
        turnTorqueCurrent = table.get("turnTorqueCurrent", turnTorqueCurrent);
    }

}
