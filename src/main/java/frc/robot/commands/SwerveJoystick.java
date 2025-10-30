package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.DriveConstants;
import frc.robot.subsystem.drive.SwerveDrive;

public class SwerveJoystick extends Command {
    private final SwerveDrive swerveDrive;
    private final XboxController joystick;

    private int counter = 100; //2 second delay

    public SwerveJoystick(SwerveDrive swerveDrive, XboxController joystick) {
        this.swerveDrive = swerveDrive;
        this.joystick = joystick;

        // User addRequirements() to declare subsystem dependencies
        addRequirements(this.swerveDrive);
    }

    @Override
    public void execute() {
        double ySpeed = joystick.getLeftY() * -1;//Y matches sim's forward backward
        double xSpeed = joystick.getLeftX() * -1;//X matches sim's left right
        // multiple by -1 to change sign
        //when using right stick and point left the sim was rotating right
        double turningSpeed = joystick.getRightX() * -1 ;

        Logger.recordOutput("Joystick/xSpeedRaw", xSpeed);
        Logger.recordOutput("Joystick/ySpeedRaw", ySpeed);
        Logger.recordOutput("Joystick/turningSpeedRaw", turningSpeed);

        //MANIPULATE JOYSTICK INPUT FOR TESTING START
        turningSpeed = 0; // IGNORE ROBOT ROTATION
        //COMMENT ALL LINES OF CODE TO FREELY TEST TURNING WITH JOYSTICK

        //LOCK LEFT AND RIGHT TO TEST FORWARD ONLY MOVEMENT
//        xSpeed = 0;//UNCOMMENT TO TEST ONLY FORWARD MOVEMENT

        //TEST HARNESS TO TEST TURNING UNCOMMENT TO USE TURN TESTING HARNESS
//        if(DriverStation.isEnabled()) {
//            if (counter == 0) {
//                xSpeed = 1; //RIGHT TURN?
//            } else {
//                counter -= 1;
//            }
//        }
        //MANIPULATE JOYSTICK INPUT FOR TESTING END

        //scale to meters per second
        xSpeed = xSpeed*DriveConstants.MAX_SPEED_METER_PER_SECONDS_DEFAULT_VALUE;
        ySpeed = ySpeed*DriveConstants.MAX_SPEED_METER_PER_SECONDS_DEFAULT_VALUE;
        turningSpeed = turningSpeed*DriveConstants.MAX_SPEED_METER_PER_SECONDS_DEFAULT_VALUE;



        //IN TELEOP WE WANT FIELD RELATIVE
        ChassisSpeeds chassisSpeed = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, turningSpeed, swerveDrive.getMeasuredAngle());
        swerveDrive.driveRobot(chassisSpeed);
    }
}
