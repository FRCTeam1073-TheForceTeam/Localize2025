// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;
import java.util.Map;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.TeleopDrive;
import frc.robot.subsystems.AprilTagFinder;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.FieldMap;
import frc.robot.subsystems.Localizer;
import frc.robot.subsystems.MapDisplay;
import frc.robot.subsystems.OI;
import frc.robot.subsystems.SwerveModuleConfig;

public class RobotContainer 
{
  private final Drivetrain m_drivetrain = new Drivetrain();
  private final OI m_OI = new OI();
  private final AprilTagFinder m_aprilTagFinder = new AprilTagFinder();
  private final Field2d m_field = new Field2d();
  private final FieldMap m_fieldMap = new FieldMap();
  private final Localizer m_localizer = new Localizer(m_drivetrain, m_fieldMap, m_aprilTagFinder);
  private final MapDisplay m_MapDisplay = new MapDisplay(m_drivetrain, m_localizer, m_fieldMap);


  private final TeleopDrive m_teleopCommand = new TeleopDrive(m_drivetrain, m_OI, m_aprilTagFinder);

  private boolean isRed;

  private Pose2d where = new Pose2d();

  public RobotContainer() 
  {
    CommandScheduler.getInstance().setDefaultCommand(m_drivetrain, m_teleopCommand);

    SmartDashboard.putData(m_drivetrain);
    SmartDashboard.putData(m_OI);
    SmartDashboard.putData("Field", m_field);

    configureBindings();
  }

  private void configureBindings() {}

  public void autonomousInit()
  {
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
    Drivetrain.initPreferences();
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

  public void disabledInit() {
    double centerY = 4.026;
    double centerX = 8.774;
    double startLineOffset = 12.227 -8.774 - 2.24; //id 10 x value - center x value - offset from reef to startline
    Pose2d startPos = new Pose2d();
    SmartDashboard.putString("Alliance", "None");

    if(DriverStation.getAlliance().isPresent())
    {
      if (DriverStation.getAlliance().get() == Alliance.Blue)
      {
        isRed = false;
        SmartDashboard.putString("Alliance", "Blue");
        startPos = new Pose2d(centerX-startLineOffset, centerY, new Rotation2d(Math.PI)); //startline
      }
      else if (DriverStation.getAlliance().get() == Alliance.Red)
      {
        SmartDashboard.putString("Alliance", "Red");
        isRed = true;
        startPos = new Pose2d(centerX + startLineOffset, centerY, new Rotation2d(0)); //startline
      }
      else
      {
        SmartDashboard.putString("Alliance", "Null");
        isRed = false;
        startPos = new Pose2d(0, 0, new Rotation2d(0));
      }
      m_drivetrain.resetOdometry(startPos);
      m_localizer.resetPos(startPos);
    }
  }

  public void disabledPeriodic() {
  
  }
}
