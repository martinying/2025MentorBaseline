// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.SwerveJoystick;
import frc.robot.constants.IOConstants;
import frc.robot.subsystem.drive.SwerveDrive;

public class RobotContainer {
  private final XboxController driveJoyStick = new XboxController(IOConstants.DRIVE_JOYSTICK_ID);
  private final SwerveDrive swerveDrive = new SwerveDrive();

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    swerveDrive.setDefaultCommand(new SwerveJoystick(swerveDrive, driveJoyStick));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
