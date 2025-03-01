// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
//import frc.robot.subsystems.AprilTagFinder;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.OI;

public class TeleopDrive extends SchemaArbiter 
{
  double angleTolerance = 0.05;
  double startAngle;
  double desiredAngle;
  ChassisSpeeds chassisSpeeds;
  Pose2d targetRotation;
  Pose2d robotRotation;
  Drivetrain m_drivetrain;
  OI m_OI;
  private boolean fieldCentric;
  private boolean parked = false;
  ChassisSpeeds speeds;
  double last_error = 0; //for snap-to-positions derivative
  double last_time = 0; //for snap-to-positions derivative
  boolean lastParkingBreakButton = false;
  boolean lastRobotCentricButton = false;
  TeleopTranslateSchema translateSchema;
  TeleopRotateSchema rotateSchema;
  boolean pointAtTarget;
  //AprilTagFinder aprilTagFinder;

  PIDController snapPidProfile;

  // Teleop drive velocity scaling:
  private final static double maximumLinearVelocity = 3.5;   // Meters/second
  private final static double maximumRotationVelocity = 4.0; // Radians/second


  /** Creates a new Teleop. */
  public TeleopDrive(Drivetrain ds, OI oi) 
  {
    super(ds, true, false);
    super.setName("Teleop Drive");
    m_drivetrain = ds;
    m_OI = oi;
    fieldCentric = true;
    startAngle = ds.getHeadingDegrees();
    desiredAngle = startAngle;
    pointAtTarget = false;
    snapPidProfile = new PIDController(
      0.05, 
      0.0, 
      0.0);
    //aprilTagFinder = finder;
    translateSchema = new TeleopTranslateSchema(m_OI, maximumLinearVelocity);
    rotateSchema = new TeleopRotateSchema(m_OI, maximumRotationVelocity);
    addSchema(translateSchema);
    addSchema(rotateSchema);
    // addSchema(turn180Schema);
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(ds);
  }

  public void initPreferences()
  {
    Preferences.initDouble("Snap to Position P", 0.1);
    Preferences.initDouble("Snap to Position I", 0);
    Preferences.initDouble("Snap to Position D", 0);
    Preferences.initDouble("Snap to Position Max Acceleration", 0.5);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize()
  {
    System.out.println("TeleopDrive: Init");
    super.initialize();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute()
  {
    
    rotateSchema.setActive(true);
    SmartDashboard.putBoolean("Rotate Schema Active", rotateSchema.getActive());

    if(m_OI.getDriverRightBumper()){
      fieldCentric = false;
      setFieldCentric(false);
    }
    else{
      fieldCentric = true;
      setFieldCentric(true);
    }
    SmartDashboard.putBoolean("Field Centric", fieldCentric);
    SmartDashboard.putBoolean("Parking Brake", parked);

    if(m_OI.getDriverLeftBumper() && lastParkingBreakButton == false)
    {
      parked = !parked;
    }
    lastParkingBreakButton = m_OI.getDriverLeftBumper();
    if(parked && !m_drivetrain.getParkingBrake())
    {
      m_drivetrain.parkingBrake(true);
    }
    if(!parked && m_drivetrain.getParkingBrake())
    {
      m_drivetrain.parkingBrake(false);
    }
    else 
    { 
      translateSchema.update(m_drivetrain);
      rotateSchema.update(m_drivetrain);
    }
    
    // Allow driver to zero the drive subsystem heading for field-centric control.
    if(m_OI.getDriverMenuButton()){
      m_drivetrain.zeroHeading();
    }

    if(m_OI.getDriverAButton()){
      Rotation2d zeroRotate = new Rotation2d();
      Pose2d zero = new Pose2d(0.0, 0.0, zeroRotate);
      m_drivetrain.resetOdometry(zero);
    }


    SmartDashboard.putBoolean("Field Centric ", fieldCentric);

    super.execute();

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted){
    if (interrupted) {
      System.out.println("TeleopDrive: Interrupted!");
    }
    super.end(interrupted);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished(){
    return false;
  }
}
