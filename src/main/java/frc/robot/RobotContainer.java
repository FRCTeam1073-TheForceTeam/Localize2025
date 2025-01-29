// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.TeleopDrive;
import frc.robot.commands.Autos.AutoCenterStart;
import frc.robot.commands.Autos.AutoLeftStart;
import frc.robot.commands.Autos.AutoRightStart;
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
  private int level;

  private final SendableChooser<String> m_positionChooser = new SendableChooser<>();
  private static final String noPositionAuto = "No Position";
  private static final String rightAuto = "Right Auto";
  private static final String leftAuto = "Left Auto";
  private static final String centerAuto = "Center Auto";
  
  private final SendableChooser<String> m_levelChooser = new SendableChooser<>();
  private static final String testLevel = "Test Level";
  private static final String noLevelAuto = "No Level";
  private static final String level1 = "Level 1";
  private static final String level2 = "Level 2";
  private static final String level3 = "Level 3";
  private static final String level4 = "Level 4";

  public RobotContainer() 
  {
    CommandScheduler.getInstance().setDefaultCommand(m_drivetrain, m_teleopCommand);

    SmartDashboard.putData(m_drivetrain);
    SmartDashboard.putData(m_OI);
    SmartDashboard.putData("Field", m_field);
    SmartDashboard.putData(m_localizer);

    m_positionChooser.setDefaultOption("No Position", noPositionAuto);
    m_positionChooser.addOption("Right Auto", rightAuto);
    m_positionChooser.addOption("Left Auto", leftAuto);
    m_positionChooser.addOption("Center Auto", centerAuto);

    m_levelChooser.setDefaultOption("No Level", noLevelAuto);
    m_levelChooser.addOption("Test Auto", testLevel);
    m_levelChooser.addOption("Level 1", level1);
    m_levelChooser.addOption("Level 2", level2);
    m_levelChooser.addOption("Level 3", level3);
    m_levelChooser.addOption("Level 4", level4);

    SmartDashboard.putData("Position Chooser", m_positionChooser);
    SmartDashboard.putData("Level Chooser", m_levelChooser);


    configureBindings();
  }

  private void configureBindings() 
  { 
    
  }

  public void autonomousInit()
  {

  }

  public Command getAutonomousCommand() 
  {
   // return Commands.print("No autonomous command configured");
   // -1 to indicate no auto select
    switch(m_levelChooser.getSelected())
    {
      case noLevelAuto:
        level = -1;
        break;
      case level1:
        level = 1;
        break;
      case level2:
        level = 2;
        break;
      case level3:
        level = 3;
        break;
      case level4:
        level = 4;                  
        break;
      case testLevel:
        level = 99;
        break;
      default:
        level = -1;
        break;
    }


    switch(m_positionChooser.getSelected())
    {
      case noPositionAuto:
        return null;
      case leftAuto:
        return AutoLeftStart.create(level, isRed, m_drivetrain);
      case rightAuto:
        return AutoRightStart.create(level, isRed, m_drivetrain, m_localizer);
      case centerAuto:
        return AutoCenterStart.create(level, isRed, m_drivetrain);
      default:
        return null;
    }
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
    //OI.initPreferences();
    //SwerveModule.initPreferences();
  }

  public Command getTeleopCommand()
  {
    return null;
  }

  public Command getDisabledCommand() 
  {
    return null;
  }

  public void disabledInit() 
  {
  }

  public boolean findStartPos() 
  {
      //create a bool for pose is set
      double centerY = 4.026;
      int allianceSign = 1;
      String selectedAuto = m_positionChooser.getSelected();
      
      double centerX = 8.774;
      double startLineOffset = 12.227 -8.774 - 2.24; //id 10 x value - center x value - offset from reef to startline
      Pose2d startPos = new Pose2d();
      SmartDashboard.putString("Alliance", "None");
  
      if(DriverStation.getAlliance().isPresent())
      {
        DriverStation.Alliance alliance = DriverStation.getAlliance().get();
        if(alliance == Alliance.Blue) {
          allianceSign = -1;
        }
        if (selectedAuto.equals(leftAuto)) {
          centerY -= allianceSign * 2.013;
        }
        else if(selectedAuto.equals(rightAuto)) {
          centerY += allianceSign * 2.013;
        }
  
        if (alliance == Alliance.Blue)
        {
          isRed = false;
          SmartDashboard.putString("Alliance", "Blue");
          startPos = new Pose2d(centerX-startLineOffset, centerY, new Rotation2d(Math.PI)); //startline
        }
        else if (alliance == Alliance.Red)
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
        m_localizer.resetPose(startPos);
        return true;
      }
      return false;
  }

  public boolean disabledPeriodic() 
  {
    return findStartPos();
  }
}
