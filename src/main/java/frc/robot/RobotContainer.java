// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.LidarAlign;
import frc.robot.commands.PrintXandY;
import frc.robot.commands.TeleopDrive;
import frc.robot.subsystems.AprilTagFinder;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.OI;
import frc.robot.subsystems.SwerveModuleConfig;
import frc.robot.subsystems.Lidar;
import frc.robot.subsystems.Localizer;

public class RobotContainer 
{
  private final Drivetrain m_drivetrain = new Drivetrain();
  private final OI m_OI = new OI();
  //private final AprilTagFinder m_aprilTagFinder = new AprilTagFinder();
  private final Lidar m_lidar = new Lidar();
  //private final PrintXandY m_print = new PrintXandY(m_lidar);
  private final LidarAlign m_lidarAlign = new LidarAlign(m_lidar, m_drivetrain);

  private final TeleopDrive m_teleopCommand = new TeleopDrive(m_drivetrain, m_OI);

  public RobotContainer() 
  {
    CommandScheduler.getInstance().setDefaultCommand(m_drivetrain, m_teleopCommand);

    configureBindings();
  }

  private void configureBindings() {
    Trigger lidarAlign = new Trigger(m_OI::getDriverBButton);
      lidarAlign.onTrue(m_lidarAlign);
    // Trigger print = new Trigger(m_OI::getDriverRightJoystick);
    //   print.whileTrue(m_print);
  }

  public Command getAutonomousCommand() 
  {
    return Commands.print("No autonomous command configured");
  }

  public void printAllFalseDiagnostics()
  {
    boolean isDisabled = DriverStation.isDisabled();
    boolean allOK = true;
    // Set allOK to the results of the printDiagnostics method for each subsystem, separated by &&
    allOK = true;
    //TODO: Add each subsystem
    SmartDashboard.putBoolean("Engine light", allOK);
  }

  public static void initPreferences()
  {
    System.out.println("RobotContainer: init Preferences.");
    SwerveModuleConfig.initPreferences();
    //Drivetrain.initPreferences();
    //OI.initPreferences();
    //SwerveModule.initPreferences();
  }

  public Command getTeleopCommand()
  {
    return null;
  }

  public Command getDisabledCommand() {
    return null;
  }

}
