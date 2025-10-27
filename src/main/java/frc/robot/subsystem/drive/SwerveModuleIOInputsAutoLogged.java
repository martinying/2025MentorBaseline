package frc.robot.subsystem.drive;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import frc.robot.subsystem.drive.SwerveModule.SwerveModuleIOInputs;

public class SwerveModuleIOInputsAutoLogged extends SwerveModuleIOInputs implements LoggableInputs {

    @Override
    public void toLog(LogTable table) {
        table.put("drive/VelocityRadPerSec", driveVelocityRadPerSec);
        table.put("drive/PositionRad", drivePositionRad);
        table.put("drive/AngularAcceleration", driveAngularAcceleration);
        table.put("drive/BridgeOutput", driveBridgeOutput);
        table.put("drive/ControlSystemTotalOutput", driveControlSystemTotalOutput);
        table.put("drive/ControlSystemProportionalOutput", driveControlSystemProportionalOutput);
        table.put("drive/ControlSystemReference", driveControlSystemReference);
        table.put("drive/DutyCycle", driveDutyCycle);
        table.put("drive/Voltage", driveVoltage);
        table.put("drive/SupplyCurrent", driveSupplyCurrent);
        table.put("drive/SupplyVoltage", driveSupplyVoltage);
        table.put("drive/TorqueCurrent", driveTorqueCurrent);
        table.put("drive/Pid/DerivativeOutput", drivePidDerivativeOutput);
        table.put("drive/Pid/IntegralOutput", drivePidIntegralOutput);
        table.put("drive/Pid/Error", drivePidError);
        table.put("drive/Pid/Output", drivePidOutput);
        table.put("drive/Pid/ProportionalOutput", drivePidProportionalOutput);
        table.put("drive/Pid/Reference", drivePidReference);
        table.put("drive/Pid/ReferenceSlope", drivePidReferenceSlope);
        
        table.put("turn/MotorControllerPosition", turnMotorControllerPosition);
        table.put("absoluteEncoderPosition", absoluteEncoderPosition);
        table.put("turn/MotorVelocityRadPerSec", turnMotorVelocityRadPerSec);
        table.put("turn/MotorAngularAcceleration", turnMotorAngularAcceleration);
        table.put("turn/BridgeOutput", turnBridgeOutput);
        table.put("turn/DutyCycle", turnDutyCycle);
        table.put("turn/Voltage", turnVoltage);
        table.put("turn/SupplyCurrent", turnSupplyCurrent);
        table.put("turn/SupplyVoltage", turnSupplyVoltage);
        table.put("turn/TorqueCurrent", turnTorqueCurrent);
    }

    @Override
    public void fromLog(LogTable table) {
        driveVelocityRadPerSec = table.get("drive/VelocityRadPerSec", driveVelocityRadPerSec);
        drivePositionRad = table.get("drive/PositionRad", drivePositionRad);
        driveAngularAcceleration = table.get("drive/AngularAcceleration", driveAngularAcceleration);
        driveBridgeOutput = table.get("drive/BridgeOutput", driveBridgeOutput);
        driveControlSystemTotalOutput = table.get("drive/ControlSystemTotalOutput", driveControlSystemTotalOutput);
        driveControlSystemProportionalOutput = table.get("drive/ControlSystemProportionalOutput", driveControlSystemProportionalOutput);
        driveControlSystemReference = table.get("drive/ControlSystemReference", driveControlSystemReference);
        driveDutyCycle = table.get("drive/DutyCycle", driveDutyCycle);
        driveVoltage = table.get("drive/Voltage", driveVoltage);
        driveSupplyCurrent = table.get("drive/SupplyCurrent", driveSupplyCurrent);
        driveSupplyVoltage = table.get("drive/SupplyVoltage", driveSupplyVoltage);
        driveTorqueCurrent = table.get("drive/TorqueCurrent", driveTorqueCurrent);
        drivePidDerivativeOutput = table.get("drive/Pid/DerivativeOutput", drivePidDerivativeOutput);
        drivePidIntegralOutput = table.get("drive/Pid/IntegralOutput", drivePidIntegralOutput);
        drivePidError = table.get("drive/Pid/Error", drivePidError);
        drivePidOutput = table.get("drive/Pid/Output", drivePidOutput);
        drivePidProportionalOutput = table.get("drive/Pid/ProportionalOutput", drivePidProportionalOutput);
        drivePidReference = table.get("drive/Pid/Reference", drivePidReference);
        drivePidReferenceSlope = table.get("drive/Pid/ReferenceSlope", drivePidReferenceSlope);

        turnMotorControllerPosition = table.get("turn/MotorControllerPosition", turnMotorControllerPosition);
        absoluteEncoderPosition = table.get("absoluteEncoderPosition", absoluteEncoderPosition);
        turnMotorVelocityRadPerSec = table.get("turn/MotorVelocityRadPerSec", turnMotorVelocityRadPerSec);
        turnMotorAngularAcceleration = table.get("turn/MotorAngularAcceleration", turnMotorAngularAcceleration);
        turnBridgeOutput = table.get("turn/BridgeOutput", turnBridgeOutput);
        turnDutyCycle = table.get("turn/DutyCycle", turnDutyCycle);
        turnVoltage = table.get("turn/Voltage", turnVoltage);
        turnSupplyCurrent = table.get("turn/SupplyCurrent", turnSupplyCurrent);
        turnSupplyVoltage = table.get("turn/SupplyVoltage", turnSupplyVoltage);
        turnTorqueCurrent = table.get("turn/TorqueCurrent", turnTorqueCurrent);
    }

}
