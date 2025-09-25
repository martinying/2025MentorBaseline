package frc.robot.subsystem;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.constants.DriveConstants;

public class SwerveModule {
    @AutoLog
    public static class SwerveModuleIOInputs {
        public double driveVelocityRadPerSec = 0.0;
        public double drivePositionRad = 0.0;
        public double driveVoltsAppliedToMotor = 0.0;
        public double driveVoltsSuppliedToMotorController = 0.0;

        public Rotation2d turnMotorControllerPosition = new Rotation2d();
        public Rotation2d absoluteEncoderPosition = new Rotation2d();
        public double turnVoltsAppliedToMotor = 0.0;
        public double turnVoltsSuppliedToMotorController = 0.0;

        public double absolutEncoderRotationsSinceLastReset = 0.0;
    }

    private final SwerveModuleIOInputs inputs = new SwerveModuleIOInputs();

    private final DutyCycleEncoder absoluteEncoder;

    private final TalonFX driveMotorController;
    private final VelocityVoltage driveVelocityInput = new VelocityVoltage(0);//default requency is 100Hz, every 10 ms

    private final TalonFX turnMotorController;
    private final PositionVoltage turnPositionInput;
    
    // Inputs from drive motor
    private final StatusSignal<AngularVelocity> driveAngularVelocity;
    private final StatusSignal<Angle> drivePosition;
    private final StatusSignal<Voltage> driveVoltsAppliedToMotor;
    private final StatusSignal<Voltage> driveVoltsSuppliedToMotorController;

    // Inputs from turn motor
    private final StatusSignal<Angle> turnPosition;
    private final StatusSignal<Voltage> turnVoltsAppliedToMotor;
    private final StatusSignal<Voltage> turnVoltsSuppliedToMotorController;
    
    public SwerveModule(int driveDeviceId, int turnDeviceId, int absoluteEncoderPort) {
        absoluteEncoder = new DutyCycleEncoder(new DigitalInput(absoluteEncoderPort), 2*Math.PI, 0);

        driveMotorController = new TalonFX(driveDeviceId);
        driveAngularVelocity = driveMotorController.getVelocity();
        drivePosition = driveMotorController.getPosition();
        driveVoltsAppliedToMotor = driveMotorController.getMotorVoltage();
        driveVoltsSuppliedToMotorController = driveMotorController.getSupplyVoltage();
        

        turnMotorController = new TalonFX(turnDeviceId);
        turnPositionInput = new PositionVoltage(absoluteEncoder.get()); //set position to what absolute encoder indicates
        turnPosition = turnMotorController.getPosition();
        turnVoltsAppliedToMotor = turnMotorController.getMotorVoltage();
        turnVoltsSuppliedToMotorController = turnMotorController.getSupplyVoltage();

        this.initDriveController(driveMotorController);
        this.initTurnController(turnMotorController);
    }

    private void initDriveController(TalonFX driveCTalonFX) {

    }

    private void initTurnController(TalonFX turnCTalonFX) {

    }

     public void setModuleState(SwerveModuleState desiredSwerveModuleStates) {
        //figures out if it only needs to do a smaller angle change and run the motor in the reverse direction
        desiredSwerveModuleStates.optimize(inputs.absoluteEncoderPosition);

        //omega (angular velocity in radians per second) = velocity/radius
        double desiredSpeedOfTheWheelInAngularVelocity = desiredSwerveModuleStates.speedMetersPerSecond/DriveConstants.WHEEL_RADIUS_DEFAULT_VALUE;
        Rotation2d desiredAngleOfTheWheel = desiredSwerveModuleStates.angle;

        Logger.recordOutput("SwerveModule/desiredSpeedOfTheWheelInAngularVelocity",desiredSpeedOfTheWheelInAngularVelocity);
        Logger.recordOutput("SwerveModuel/desiredAngleOfTheWheel", desiredAngleOfTheWheel);

        turnMotorController.setControl(turnPositionInput.withPosition(desiredAngleOfTheWheel.getRotations()));//expect rotations
        driveMotorController.setControl(driveVelocityInput.withVelocity(Units.radiansToRotations(desiredSpeedOfTheWheelInAngularVelocity)));//expect rotations per second

        Logger.recordOutput("SwerveModule/driveMCFaultField", driveMotorController.getFaultField().getValue());
        Logger.recordOutput("SwerveModule/turnMCFaultField", turnMotorController.getFaultField().getValue());
     }

    public void updateInputs() {
        BaseStatusSignal.refreshAll(driveAngularVelocity, drivePosition, driveVoltsAppliedToMotor, driveVoltsSuppliedToMotorController, turnPosition, turnVoltsAppliedToMotor, turnVoltsSuppliedToMotorController);

        inputs.driveVelocityRadPerSec=Units.rotationsToRadians(driveAngularVelocity.getValueAsDouble());
        inputs.drivePositionRad = Units.rotationsToRadians(drivePosition.getValueAsDouble());
        inputs.driveVoltsAppliedToMotor=driveVoltsAppliedToMotor.getValueAsDouble();
        inputs.driveVoltsSuppliedToMotorController=driveVoltsSuppliedToMotorController.getValueAsDouble();
        
        //this is the module's angle measured from the motor controller's onboard relative encoder
        inputs.turnMotorControllerPosition=Rotation2d.fromRotations(turnPosition.getValueAsDouble());
        inputs.turnVoltsAppliedToMotor=turnVoltsAppliedToMotor.getValueAsDouble();
        inputs.turnVoltsSuppliedToMotorController=turnVoltsSuppliedToMotorController.getValueAsDouble();

        inputs.absoluteEncoderPosition = new Rotation2d(absoluteEncoder.get());
    }
}
