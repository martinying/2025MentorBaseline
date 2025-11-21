package frc.robot.subsystem.drive;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.BridgeOutputValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.constants.DriveConstants;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

public class SwerveModule {

    @AutoLog
    public static class SwerveModuleIOInputs {
        public double driveVelocityRadPerSec = 0.0;
        public double drivePositionRad = 0.0;
        public double driveAngularAcceleration = 0.0;
        public double driveBridgeOutput = 0.0;
        public double driveControlSystemTotalOutput = 0.0;
        public double driveControlSystemProportionalOutput = 0.0;
        public double driveControlSystemReference = 0.0;
        public double driveDutyCycle = 0.0;
        public double driveVoltage = 0.0;
        public double driveSupplyCurrent = 0.0;
        public double driveSupplyVoltage = 0.0;
        public double driveTorqueCurrent = 0.0;
        public double drivePidDerivativeOutput = 0.0;
        public double drivePidIntegralOutput = 0.0;
        public double drivePidError = 0.0;
        public double drivePidOutput = 0.0;
        public double drivePidProportionalOutput = 0.0;
        public double drivePidReference = 0.0;
        public double drivePidReferenceSlope = 0.0;

        public Rotation2d turnMotorControllerRelativeEncoderPosition = new Rotation2d();
        public double turnMotorVelocityRadPerSec = 0.0;
        public double turnMotorAngularAcceleration = 0.0;
        public double turnBridgeOutput = 0.0;
        public double turnDutyCycle = 0.0;
        public double turnVoltage = 0.0;
        public double turnSupplyCurrent = 0.0;
        public double turnSupplyVoltage = 0.0;
        public double turnTorqueCurrent = 0.0;

        public double absoluteEncoderDutyCyclePosition = 0.0;
        public double absoluteEncoderPositionInDegrees = 0.0;
        public double absoluteEncoderPositionInRadians = 0.0;
    }

    private final SwerveModuleIOInputsAutoLogged inputs = new SwerveModuleIOInputsAutoLogged();

    private final DutyCycleEncoder absoluteEncoder;
    //The offset is not in terms or radians or degrees, it should be less than the encoder max value of 1
    private final double absoluteEncoderOffset;

    private final TalonFX driveMotorController;
    private final VelocityVoltage driveVelocityInput = new VelocityVoltage(0);//default frequency is 100Hz, every 10 ms

    private final TalonFX turnMotorController;
    private final DutyCycleOut turnMotorControllerInput = new DutyCycleOut(0);
    
    // Inputs from drive motor
    private final StatusSignal<AngularVelocity> driveAngularVelocity;
    private final StatusSignal<Angle> drivePosition;
    private final StatusSignal<AngularAcceleration> driveAngularAcceleration;
    //this is the electrical output from the motor controller to the motor
    private final StatusSignal<BridgeOutputValue> driveBridgeOutput;
    private final StatusSignal<Double> driveControlSystemTotalOutput;
    //this is the proportion of the error between the desired and the actual of a controlled variable
    private final StatusSignal<Double> driveControlSystemProportionalOutput;
    private final StatusSignal<Double> driveControlSystemReference; //value control system is targeting
    private final StatusSignal<Double> driveDutyCycle;
    private final StatusSignal<Voltage> driveVoltage;
    private final StatusSignal<Current> driveSupplyCurrent;
    private final StatusSignal<Voltage> driveSupplyVoltage;
    private final StatusSignal<Current> driveTorqueCurrent;
    private final StatusSignal<Double> drivePidDerivativeOutput;
    private final StatusSignal<Double> drivePidIntegralOutput;
    private final StatusSignal<Double> drivePidError;
    private final StatusSignal<Double> drivePidOutput;
    private final StatusSignal<Double> drivePidProportionalOutput;
    private final StatusSignal<Double> drivePidReference;
    private final StatusSignal<Double> drivePidReferenceSlope;

    // Inputs from turn motor
    private final StatusSignal<AngularVelocity> turnAngularVelocity;
    private final StatusSignal<Angle> turnRelativeEncoderPosition;
    private final StatusSignal<AngularAcceleration> turnAcceleration;
    //this is the electrical output from the motor controller to the motor
    private final StatusSignal<BridgeOutputValue> turnBridgeOutput;
    private final StatusSignal<Double> turnDutyCycle;
    private final StatusSignal<Voltage> turnVoltage;
    private final StatusSignal<Current> turnSupplyCurrent;
    private final StatusSignal<Voltage> turnSupplyVoltage;
    private final StatusSignal<Current> turnTorqueCurrent;

    PIDController turnPIDController = new PIDController(0.1, 0, 0);

    public SwerveModule(int driveDeviceId, int turnDeviceId, int absoluteEncoderPort, double absoluteEncoderOffset) {
        absoluteEncoder = new DutyCycleEncoder(new DigitalInput(absoluteEncoderPort));
        //The offset is not in terms or radians or degrees, it should be less than the encoder max value of 1
        this.absoluteEncoderOffset = absoluteEncoderOffset;

        driveMotorController = new TalonFX(driveDeviceId);
        driveAngularVelocity = driveMotorController.getVelocity();
        drivePosition = driveMotorController.getPosition();
        driveAngularAcceleration = driveMotorController.getAcceleration();
        driveBridgeOutput = driveMotorController.getBridgeOutput();
        driveControlSystemTotalOutput = driveMotorController.getClosedLoopOutput();
        driveControlSystemProportionalOutput = driveMotorController.getClosedLoopProportionalOutput();
        driveControlSystemReference = driveMotorController.getClosedLoopReference();
        driveDutyCycle = driveMotorController.getDutyCycle();
        driveVoltage = driveMotorController.getMotorVoltage();
        driveSupplyCurrent = driveMotorController.getSupplyCurrent();
        driveSupplyVoltage = driveMotorController.getSupplyVoltage();
        driveTorqueCurrent = driveMotorController.getTorqueCurrent();
        drivePidDerivativeOutput = driveMotorController.getClosedLoopDerivativeOutput();
        drivePidIntegralOutput = driveMotorController.getClosedLoopIntegratedOutput();
        drivePidError = driveMotorController.getClosedLoopError();
        drivePidOutput = driveMotorController.getClosedLoopOutput();
        drivePidProportionalOutput = driveMotorController.getClosedLoopProportionalOutput();
        drivePidReference = driveMotorController.getClosedLoopReference();
        drivePidReferenceSlope = driveMotorController.getClosedLoopReferenceSlope();

        turnMotorController = new TalonFX(turnDeviceId);
        turnRelativeEncoderPosition = turnMotorController.getPosition();
        turnAngularVelocity = turnMotorController.getVelocity();
        turnAcceleration = turnMotorController.getAcceleration();
        turnBridgeOutput = turnMotorController.getBridgeOutput();
        turnDutyCycle = turnMotorController.getDutyCycle();
        turnVoltage = turnMotorController.getMotorVoltage();
        turnSupplyCurrent = turnMotorController.getSupplyCurrent();
        turnSupplyVoltage = turnMotorController.getSupplyVoltage();
        turnTorqueCurrent = turnMotorController.getTorqueCurrent();

        this.initDriveControllerPID(driveMotorController);
    }

    private void initDriveControllerPID(TalonFX driveCTalonFX) {
        Slot0Configs slot0Configs = new Slot0Configs();
        slot0Configs.kP = 2.4; // An error of 1 rotation results in 2.4 V output
        slot0Configs.kI = 0; // no output for integrated error
        slot0Configs.kD = 0.1; // A velocity of 1 rps results in 0.1 V output

        driveCTalonFX.getConfigurator().apply(slot0Configs);

        Slot1Configs slot1Configs = new Slot1Configs();
        slot1Configs.kP = 0.1; // An error of 1 rotation results in 2.4 V output
        slot1Configs.kI = 0; // no output for integrated error
        slot1Configs.kD = 0.01; // A velocity of 1 rps results in 0.1 V output

        driveCTalonFX.getConfigurator().apply(slot1Configs);
    }

    private void initDriveControllerMotionMagic(TalonFX driveCTalonFX) {
        TalonFXConfiguration talonFXConfiguration = new TalonFXConfiguration();

        Slot0Configs slot0Configs = talonFXConfiguration.Slot0;
        slot0Configs.kS = 0.25; // Add 0.25 V output to overcome static friction
        slot0Configs.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
        slot0Configs.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
        slot0Configs.kP = 0.11; //

        slot0Configs.kI = 0; // no output for integrated error
        slot0Configs.kD = 0; // no output for error derivative

        MotionMagicConfigs motionMagicConfigs = talonFXConfiguration.MotionMagic;
        motionMagicConfigs.MotionMagicAcceleration = 400; // Target acceleration of 400 rps/s (0.25 seconds to max)
        motionMagicConfigs.MotionMagicJerk = 4000; // Target jerk of 4000 rps/s/s (0.1 seconds)
        
        driveCTalonFX.getConfigurator().apply(talonFXConfiguration);
    }

    private void initDriveControllerMotionMagicExpo(TalonFX driveCTalonFX) {

    }

    public void setModuleState(SwerveModuleState commandedSwerveModuleStates, int moduleIndex) {
        //to work with the most current data, get the measurements again and log them before using them in this method
        this.updateInputs();
        String proccessInputsKey = "ReplayInputs/SwerveModule/";
        switch ( moduleIndex ) {
            case DriveConstants.FRONT_LEFT_MODULE_INDEX:
                proccessInputsKey += "FrontLeft/";
                break;
            case DriveConstants.FRONT_RIGHT_MODULE_INDEX:
                proccessInputsKey += "FrontRight/";
                break;
            case DriveConstants.BACK_LEFT_MODULE_INDEX:
                proccessInputsKey += "BackLeft/";
                break;
            case DriveConstants.BACK_RIGHT_MODULE_INDEX:
                proccessInputsKey += "BackRight/";
                break;
        }
        Logger.processInputs(proccessInputsKey,getInputs());//log updated measured values

        //figures out if it only needs to do a smaller angle change and run the motor in the reverse direction
        commandedSwerveModuleStates.optimize(new Rotation2d(inputs.absoluteEncoderPositionInRadians));

        //DRIVE MOTOR CALCULATIONS
        //omega (angular velocity in radians per second) = velocity/radius
        double wheelRotationsPerSec = commandedSwerveModuleStates.speedMetersPerSecond/(DriveConstants.WHEEL_DIAMETER_IN_METERS*Math.PI);
        double commandedMotorRotationsPerSec = wheelRotationsPerSec*DriveConstants.SWERVE_MODULE_DRIVE_MOTOR_GEAR_RATIO;

        //determine which drive motor pid to use, test mode vs. competition mode
        int driveMotorControllerSlotNumber = 0;
        if(DriverStation.isTest()) {//set drive and turn pid
            turnPIDController.setP(0.01);
            driveMotorControllerSlotNumber = 1;
        } else {
            turnPIDController.setP(0.4);
            //the default slot 0 is used for drive pid
        }

        //log module information
        logModuleState(moduleIndex, commandedMotorRotationsPerSec, commandedSwerveModuleStates);

        turnMotorController.setControl(
                turnMotorControllerInput.withOutput(getTurnMotorCalculatedPidValue(commandedSwerveModuleStates)));

        //driveMotorController.setControl(driveVelocityInput.withSlot(driveMotorControllerSlotNumber).withVelocity(0));
        driveMotorController.setControl(driveVelocityInput.withSlot(driveMotorControllerSlotNumber).withVelocity(commandedMotorRotationsPerSec));
    }

    private double getTurnMotorCalculatedPidValue(SwerveModuleState commandedSwerveModuleStates) {
        //the commandedSwerveModuleStates.angle is in the range of -Pi to Pi
        //angleModulus will convert 0 to 2Pi to -Pi to Pi
        return turnPIDController.calculate(MathUtil.angleModulus(inputs.absoluteEncoderPositionInRadians), commandedSwerveModuleStates.angle.getRadians());
    }

    private void logModuleState(int moduleIndex, double commandedMotorRotationsPerSec, SwerveModuleState commandedSwerveModuleState) {
        String loggerKeyPrefix = "SwerveModule/";
        switch ( moduleIndex) {
            case DriveConstants.FRONT_LEFT_MODULE_INDEX:
                loggerKeyPrefix += "FrontLeft/";
                break;
            case DriveConstants.FRONT_RIGHT_MODULE_INDEX:
                loggerKeyPrefix += "FrontRight/";
                break;
            case DriveConstants.BACK_LEFT_MODULE_INDEX:
                loggerKeyPrefix += "BackLeft/";
                break;
            case DriveConstants.BACK_RIGHT_MODULE_INDEX:
                loggerKeyPrefix += "BackRight/";
                break;
        }

        //log absolute encoder information
        Logger.recordOutput(loggerKeyPrefix+"/absoluteEncoder/dutyCyclePosition", inputs.absoluteEncoderDutyCyclePosition); //0 to 1
        Logger.recordOutput(loggerKeyPrefix+"/absoluteEncoder/absoluteEncoderDutyCycleOffset", absoluteEncoderOffset);
        Logger.recordOutput(loggerKeyPrefix+"/absoluteEncoder/absoluteEncoderPositionInDegrees", inputs.absoluteEncoderPositionInDegrees);
        Logger.recordOutput(loggerKeyPrefix+"/absoluteEncoder/absoluteEncoderPositionInRadians", inputs.absoluteEncoderPositionInRadians);
        Logger.recordOutput(loggerKeyPrefix+"/absoluteEncoder/encoderGet", absoluteEncoder.get());
        Logger.recordOutput(loggerKeyPrefix+"/absoluteEncoder/offset", absoluteEncoderOffset);


        //log drive motor information
        Logger.recordOutput(loggerKeyPrefix+"drive/commandedMotorRotationsPerSec",commandedMotorRotationsPerSec);

        //log turn motor information
        Logger.recordOutput(loggerKeyPrefix+"turn/commandedAngleOfTheWheelInDegrees", commandedSwerveModuleState.angle);
        Logger.recordOutput(loggerKeyPrefix+"turn/Pid/errorDerivative", turnPIDController.getErrorDerivative());
        Logger.recordOutput(loggerKeyPrefix+"turn/Pid/error", turnPIDController.getError());
        Logger.recordOutput(loggerKeyPrefix+"turn/Pid/accumulatedError", turnPIDController.getAccumulatedError());
        Logger.recordOutput(loggerKeyPrefix+"turn/Pid/errorTolerance", turnPIDController.getErrorTolerance());
        Logger.recordOutput(loggerKeyPrefix+"turn/Pid/errorDerivativeTolerance", turnPIDController.getErrorDerivativeTolerance());
        Logger.recordOutput(loggerKeyPrefix+"turn/Pid/iZone", turnPIDController.getIZone());
        Logger.recordOutput(loggerKeyPrefix+"turn/Pid/calculatedPidValue", getTurnMotorCalculatedPidValue(commandedSwerveModuleState));
        Logger.recordOutput(loggerKeyPrefix+"turn/Pid/kP", turnPIDController.getP());
    }

    public void updateInputs() {
        BaseStatusSignal.refreshAll(
                driveAngularVelocity, drivePosition, driveAngularAcceleration,
                driveBridgeOutput, driveControlSystemTotalOutput,
                driveControlSystemProportionalOutput, driveControlSystemReference,
                driveDutyCycle, driveVoltage, driveSupplyCurrent, driveSupplyVoltage,
                driveTorqueCurrent, drivePidDerivativeOutput, drivePidIntegralOutput,
                drivePidError, drivePidOutput, drivePidProportionalOutput,
                drivePidReference, drivePidReferenceSlope,
                turnRelativeEncoderPosition, turnAngularVelocity, turnAcceleration,
                turnBridgeOutput, turnDutyCycle, turnVoltage, turnSupplyCurrent,
                turnSupplyVoltage, turnTorqueCurrent);

        inputs.driveVelocityRadPerSec=Units.rotationsToRadians(driveAngularVelocity.getValueAsDouble());
        inputs.drivePositionRad = Units.rotationsToRadians(drivePosition.getValueAsDouble());
        inputs.driveAngularAcceleration = driveAngularAcceleration.getValueAsDouble();
        inputs.driveBridgeOutput = driveBridgeOutput.getValueAsDouble();
        inputs.driveControlSystemTotalOutput = driveControlSystemTotalOutput.getValueAsDouble();
        inputs.driveControlSystemProportionalOutput = driveControlSystemProportionalOutput.getValueAsDouble();
        inputs.driveControlSystemReference = driveControlSystemReference.getValueAsDouble();
        inputs.driveDutyCycle = driveDutyCycle.getValueAsDouble();
        inputs.driveVoltage = driveVoltage.getValueAsDouble();
        inputs.driveSupplyCurrent = driveSupplyCurrent.getValueAsDouble();
        inputs.driveSupplyVoltage = driveSupplyVoltage.getValueAsDouble();
        inputs.driveTorqueCurrent = driveTorqueCurrent.getValueAsDouble();
        inputs.drivePidDerivativeOutput = drivePidDerivativeOutput.getValueAsDouble();
        inputs.drivePidIntegralOutput = drivePidIntegralOutput.getValueAsDouble();
        inputs.drivePidError = drivePidError.getValueAsDouble();
        inputs.drivePidOutput = drivePidOutput.getValueAsDouble();
        inputs.drivePidProportionalOutput = drivePidProportionalOutput.getValueAsDouble();
        inputs.drivePidReference = drivePidReference.getValueAsDouble();
        inputs.drivePidReferenceSlope = drivePidReferenceSlope.getValueAsDouble();
        
        //this is the module's angle measured from the motor controller's onboard relative encoder
        //we may not need this value since it's confusing
        inputs.turnMotorControllerRelativeEncoderPosition =Rotation2d.fromRotations(turnRelativeEncoderPosition.getValueAsDouble());
        inputs.turnMotorAngularAcceleration = turnAcceleration.getValueAsDouble();
        inputs.turnMotorVelocityRadPerSec = Units.rotationsToRadians(turnAngularVelocity.getValueAsDouble());
        inputs.turnBridgeOutput = turnBridgeOutput.getValueAsDouble();
        inputs.turnDutyCycle = turnDutyCycle.getValueAsDouble();
        inputs.turnVoltage = turnVoltage.getValueAsDouble();
        inputs.turnSupplyCurrent = turnSupplyCurrent.getValueAsDouble();
        inputs.turnSupplyVoltage = turnSupplyVoltage.getValueAsDouble();
        inputs.turnTorqueCurrent = turnTorqueCurrent.getValueAsDouble();

        //get the absolute encoder position for both calculations to eliminate any drift between two separate calls
        //the absolute encoder returns a value from 0-1
        inputs.absoluteEncoderDutyCyclePosition = absoluteEncoder.get()-absoluteEncoderOffset;//handle offset
        //to convert 0-1 to degrees, we multiply it by 360
        inputs.absoluteEncoderPositionInDegrees = inputs.absoluteEncoderDutyCyclePosition*360;
        //to convert 0-1 to radians, we multiply it by 2*PI
        inputs.absoluteEncoderPositionInRadians = inputs.absoluteEncoderDutyCyclePosition*2*Math.PI;
    }

    public SwerveModuleIOInputsAutoLogged getInputs() {
        return inputs;
    }
}
