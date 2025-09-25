package frc.robot.commands;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.DriveConstants;
import frc.robot.subsystem.drive.SwerveDrive;

public class SwerveJoystick extends Command {
    private final SwerveDrive swerveDrive;
    private final XboxController joystick;

    public SwerveJoystick(SwerveDrive swerveDrive, XboxController joystick) {
        this.swerveDrive = swerveDrive;
        this.joystick = joystick;

        // User addRequirements() to declare subsystem dependencies
        addRequirements(this.swerveDrive);
    }

    @Override
    public void execute() {
        double xSpeed = joystick.getLeftY() * -1;//Y matches sim's foward backward
        double ySpeed = joystick.getLeftX() * -1;//X matches sim's left right
        // multiple by -1 to change sign
        //when using right stick and point left the sim was rotating right
        double turningSpeed = joystick.getRightX() * -1 ;

        Logger.recordOutput("Joystick/xSpeedRaw", xSpeed);
        Logger.recordOutput("Joystick/ySpeedRaw", ySpeed);
        Logger.recordOutput("Joystick/turningSpeedRaw", turningSpeed);

        //scale to meters per second
        xSpeed = xSpeed*DriveConstants.MAX_SPEED_METER_PER_SECCONDS_DEFAULT_VALUE;
        ySpeed = ySpeed*DriveConstants.MAX_SPEED_METER_PER_SECCONDS_DEFAULT_VALUE;
        turningSpeed = turningSpeed*DriveConstants.MAX_SPEED_METER_PER_SECCONDS_DEFAULT_VALUE;

        //IN TELEOP WE WANT FIELD RELATIVE
        ChassisSpeeds chassisSpeed = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, turningSpeed, swerveDrive.getMeasuredAngle());
        swerveDrive.driveRobot(chassisSpeed);
    }
}
