package frc.robot.subsystem;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.BridgeOutputValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.RobotBase;
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

        public Rotation2d turnMotorControllerPosition = new Rotation2d();
        public double turnMotorVelocityRadPerSec = 0.0;
        public double turnMotorAngularAcceleration = 0.0;
        public double turnBridgeOutput = 0.0;
        public double turnDutyCycle = 0.0;
        public double turnVoltage = 0.0;
        public double turnSupplyCurrent = 0.0;
        public double turnSupplyVoltage = 0.0;
        public double turnTorqueCurrent = 0.0;

        public Rotation2d absoluteEncoderPosition = new Rotation2d();
    }

    private final SwerveModuleIOInputsAutoLogged inputs = new SwerveModuleIOInputsAutoLogged();

    private final DutyCycleEncoder absoluteEncoder;

    private final TalonFX driveMotorController;
    private final VelocityVoltage driveVelocityInput = new VelocityVoltage(0);//default frequency is 100Hz, every 10 ms

    private final TalonFX turnMotorController;
    private final PositionVoltage turnPositionInput;
    
    // Inputs from drive motor
    private final StatusSignal<AngularVelocity> driveAngularVelocity;
    private final StatusSignal<Angle> drivePosition;
    private final StatusSignal<AngularAcceleration> driveAngularAcceleration;
    //this is the electrical output from the motor controller to the motor
    private StatusSignal<BridgeOutputValue> driveBridgeOutput;
    private StatusSignal<Double> driveControlSystemTotalOutput;
    //this is the proportion of the error between the desired and the actual of a controlled variable
    private StatusSignal<Double> driveControlSystemProportionalOutput;
    private StatusSignal<Double> driveControlSystemReference; //value control system is targeting
    private StatusSignal<Double> driveDutyCycle;
    private StatusSignal<Voltage> driveVoltage;
    private StatusSignal<Current> driveSupplyCurrent;
    private StatusSignal<Voltage> driveSupplyVoltage;
    private StatusSignal<Current> driveTorqueCurrent;

    // Inputs from turn motor
    private final StatusSignal<AngularVelocity> turnAngularVelocity;
    private final StatusSignal<Angle> turnPosition;
    private final StatusSignal<AngularAcceleration> turnAcceleration;
    //this is the electrical output from the motor controller to the motor
    private StatusSignal<BridgeOutputValue> turnBridgeOutput;
    private StatusSignal<Double> turnDutyCycle;
    private StatusSignal<Voltage> turnVoltage;
    private StatusSignal<Current> turnSupplyCurrent;
    private StatusSignal<Voltage> turnSupplyVoltage;
    private StatusSignal<Current> turnTorqueCurrent;
    
    public SwerveModule(int driveDeviceId, int turnDeviceId, int absoluteEncoderPort) {
        absoluteEncoder = new DutyCycleEncoder(new DigitalInput(absoluteEncoderPort), 2*Math.PI, 0);

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

        turnMotorController = new TalonFX(turnDeviceId);
        turnPositionInput = new PositionVoltage(absoluteEncoder.get()); //set position to what absolute encoder indicates
        turnPosition = turnMotorController.getPosition();
        turnAngularVelocity = turnMotorController.getVelocity();
        turnAcceleration = turnMotorController.getAcceleration();
        turnBridgeOutput = turnMotorController.getBridgeOutput();
        turnDutyCycle = turnMotorController.getDutyCycle();
        turnVoltage = turnMotorController.getMotorVoltage();
        turnSupplyCurrent = turnMotorController.getSupplyCurrent();
        turnSupplyVoltage = turnMotorController.getSupplyVoltage();
        turnTorqueCurrent = turnMotorController.getTorqueCurrent();

        this.initDriveControllerPID(driveMotorController);
        this.initTurnControllerPID(turnMotorController);
    }

    private void initDriveControllerPID(TalonFX driveCTalonFX) {
        Slot0Configs config = new Slot0Configs();
        config.kP = 2.4; // An error of 1 rotation results in 2.4 V output
        config.kI = 0; // no output for integrated error
        config.kD = 0.1; // A velocity of 1 rps results in 0.1 V output

        driveCTalonFX.getConfigurator().apply(config);
    }

    private void initDriveControllerMotionMagic(TalonFX driveCTalonFX) {
        TalonFXConfiguration talonFXConfiguration = new TalonFXConfiguration();

        Slot0Configs slot0Configs = talonFXConfiguration.Slot0;
        slot0Configs.kS = 0.25; // Add 0.25 V output to overcome static friction
        slot0Configs.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
        slot0Configs.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
        slot0Configs.kP = 0.11; // An error of 1 rps results in 0.11 V output
        slot0Configs.kI = 0; // no output for integrated error
        slot0Configs.kD = 0; // no output for error derivative

        MotionMagicConfigs motionMagicConfigs = talonFXConfiguration.MotionMagic;
        motionMagicConfigs.MotionMagicAcceleration = 400; // Target acceleration of 400 rps/s (0.25 seconds to max)
        motionMagicConfigs.MotionMagicJerk = 4000; // Target jerk of 4000 rps/s/s (0.1 seconds)
        
        driveCTalonFX.getConfigurator().apply(talonFXConfiguration);
    }

    private void initDriveControllerMotionMagicExpo(TalonFX driveCTalonFX) {

    }

    private void initTurnControllerPID(TalonFX turnCTalonFX) {
        Slot0Configs config = new Slot0Configs();
        config.kP = 2.4; // An error of 1 rotation results in 2.4 V output
        config.kI = 0; // no output for integrated error
        config.kD = 0.1; // A velocity of 1 rps results in 0.1 V output

        turnCTalonFX.getConfigurator().apply(config);
    }

    private void initTurnControllerMotionMagic(TalonFX turnCTalonFX) {
        TalonFXConfiguration talonFXConfiguration = new TalonFXConfiguration();

        Slot0Configs slot0Configs = talonFXConfiguration.Slot0;
        slot0Configs.kS = 0.25; // Add 0.25 V output to overcome static friction
        slot0Configs.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
        slot0Configs.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
        slot0Configs.kP = 4.8; // A position error of 2.5 rotations results in 12 V output
        slot0Configs.kI = 0; // no output for integrated error
        slot0Configs.kD = 0.1; // A velocity error of 1 rps results in 0.1 V output
        
        MotionMagicConfigs motionMagicConfigs = talonFXConfiguration.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = 80; // Target cruise velocity of 80 rps
        motionMagicConfigs.MotionMagicAcceleration = 160; // Target acceleration of 160 rps/s (0.5 seconds)
        motionMagicConfigs.MotionMagicJerk = 1600; // Target jerk of 1600 rps/s/s (0.1 seconds)

        turnCTalonFX.getConfigurator().apply(talonFXConfiguration);
    }

    private void initTurnControllerMotionMagicExpo(TalonFX turnCTalonFX) {
        TalonFXConfiguration talonFXConfiguration = new TalonFXConfiguration();

        Slot0Configs slot0Configs = talonFXConfiguration.Slot0;
        slot0Configs.kS = 0.25; // Add 0.25 V output to overcome static friction
        slot0Configs.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
        slot0Configs.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
        slot0Configs.kP = 4.8; // A position error of 2.5 rotations results in 12 V output
        slot0Configs.kI = 0; // no output for integrated error
        slot0Configs.kD = 0.1; // A velocity error of 1 rps results in 0.1 V output
        
        MotionMagicConfigs motionMagicConfigs = talonFXConfiguration.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = 0; // Unlimited cruise velocity
        motionMagicConfigs.MotionMagicExpo_kV = 0.12; // kV is around 0.12 V/rps
        motionMagicConfigs.MotionMagicExpo_kA = 0.1; // Use a slower kA of 0.1 V/(rps/s)
        
        turnCTalonFX.getConfigurator().apply(talonFXConfiguration);
    }

     public void setModuleState(SwerveModuleState desiredSwerveModuleStates, int moduleIndex) {
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

        //figures out if it only needs to do a smaller angle change and run the motor in the reverse direction
        desiredSwerveModuleStates.optimize(inputs.absoluteEncoderPosition);

        //omega (angular velocity in radians per second) = velocity/radius
        double desiredSpeedOfTheWheelInAngularVelocity = desiredSwerveModuleStates.speedMetersPerSecond/(DriveConstants.WHEEL_DIAMETER_IN_METERS*Math.PI);
        Rotation2d desiredAngleOfTheWheel = desiredSwerveModuleStates.angle;

        Logger.recordOutput(loggerKeyPrefix+"desiredSpeedOfTheWheelInAngularVelocity",desiredSpeedOfTheWheelInAngularVelocity);
        Logger.recordOutput(loggerKeyPrefix+"desiredAngleOfTheWheel", desiredAngleOfTheWheel);

        turnMotorController.setControl(turnPositionInput.withPosition(desiredAngleOfTheWheel.getRotations()));//expect rotations
        driveMotorController.setControl(driveVelocityInput.withVelocity(Units.radiansToRotations(desiredSpeedOfTheWheelInAngularVelocity)));//expect rotations per second

        Logger.recordOutput(loggerKeyPrefix+"driveMCFaultField", driveMotorController.getFaultField().getValue());
        Logger.recordOutput(loggerKeyPrefix+"turnMCFaultField", turnMotorController.getFaultField().getValue());
     }

    public void updateInputs() {
        BaseStatusSignal.refreshAll(
                driveAngularVelocity, drivePosition, driveAngularAcceleration,
                driveBridgeOutput,
                turnPosition, turnAngularVelocity, turnAcceleration);

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
        
        //this is the module's angle measured from the motor controller's onboard relative encoder
        inputs.turnMotorControllerPosition=Rotation2d.fromRotations(turnPosition.getValueAsDouble());
        inputs.turnMotorAngularAcceleration = turnAcceleration.getValueAsDouble();
        inputs.turnMotorVelocityRadPerSec = Units.rotationsToRadians(turnAngularVelocity.getValueAsDouble());
        inputs.turnBridgeOutput = turnBridgeOutput.getValueAsDouble();
        inputs.turnDutyCycle = turnDutyCycle.getValueAsDouble();
        inputs.turnVoltage = turnVoltage.getValueAsDouble();
        inputs.turnSupplyCurrent = turnSupplyCurrent.getValueAsDouble();
        inputs.turnSupplyVoltage = turnSupplyVoltage.getValueAsDouble();
        inputs.turnTorqueCurrent = turnTorqueCurrent.getValueAsDouble();

        inputs.absoluteEncoderPosition = new Rotation2d(absoluteEncoder.get());
    }

    public SwerveModuleIOInputsAutoLogged getInputs() {
        return inputs;
    }
}
