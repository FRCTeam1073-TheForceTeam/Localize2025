// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.AprilTagFinder;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Localizer;
import frc.robot.subsystems.OI;

public class TeleopDrive extends Command 
{
  double angleTolerance = 0.05;
  double startAngle;
  double desiredAngle;
  ChassisSpeeds chassisSpeeds;
  Pose2d Rotation;
  Pose2d robotRotation;
  Drivetrain drivetrain;
  OI m_OI;
  private boolean fieldCentric;
  private boolean parked = false;
  ChassisSpeeds speeds;
  double last_error = 0; //for snap-to-positions derivative
  double last_time = 0; //for snap-to-positions derivative
  boolean lastParkingBreakButton = false;
  boolean lastRobotCentricButton = false;
  boolean pointAtTarget;
  AprilTagFinder aprilTagFinder;
  Localizer localizer;

  PIDController snapPidProfile;

  // Teleop drive velocity scaling:
  private final static double maximumLinearVelocity = 3.5;   // Meters/second
  private final static double maximumRotationVelocity = 4.0; // Radians/second
  private double mult1;
  private double mult2;
  private double leftX;
  private double leftY;
  private double rightX;
  private double vx;
  private double vy;
  private double w;


  /** Creates a new Teleop. */
  public TeleopDrive(Drivetrain drivetrain, OI oi, AprilTagFinder finder, Localizer localizer) 
  {
  
    this.drivetrain = drivetrain;
    this.localizer = localizer;
    m_OI = oi;
    fieldCentric = true;
    startAngle = drivetrain.getHeadingDegrees();
    desiredAngle = startAngle;
    pointAtTarget = false;
    snapPidProfile = new PIDController(
      0.05, 
      0.0, 
      0.0);
    aprilTagFinder = finder;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
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
    leftY = m_OI.getDriverTranslateY();
    leftX = m_OI.getDriverTranslateX();
    rightX = m_OI.getDriverRotate();

    SmartDashboard.putBoolean("Parking Brake", parked);

    if(m_OI.getDriverLeftBumper() && lastParkingBreakButton == false)
    {
      parked = !parked;
    }
    lastParkingBreakButton = m_OI.getDriverLeftBumper();
    if(parked && !drivetrain.getParkingBrake())
    {
      drivetrain.parkingBrake(true);
    }
    if(!parked && drivetrain.getParkingBrake())
    {
      drivetrain.parkingBrake(false);
    }
    else 
    { 
      //multiples the angle by a number from 1 to the square root of 30:
        mult1 = 1.0 + (m_OI.getDriverLeftTrigger() * ((Math.sqrt(25)) - 1));
        mult2 = 1.0 + (m_OI.getDriverRightTrigger() * ((Math.sqrt(25)) - 1));

        

        //sets deadzones on the controller to extend to .05:
        if(Math.abs(leftY) < .15) {leftY = 0;}
        if(Math.abs(leftX) < .15) {leftX = 0;}
        if(Math.abs(rightX) < .15) {rightX = 0;}

        vx = MathUtil.clamp((-leftY * maximumLinearVelocity / 25 ) * mult1 * mult2, -maximumLinearVelocity, maximumLinearVelocity);
        vy = MathUtil.clamp((-leftX * maximumLinearVelocity / 25 ) * mult1 * mult2, -maximumLinearVelocity, maximumLinearVelocity);
        w = MathUtil.clamp(-(rightX * maximumRotationVelocity / 25) * mult1 * mult2, -maximumRotationVelocity, maximumRotationVelocity);

        SmartDashboard.putNumber("TeleopDrive/vx", vx);

        drivetrain.setTargetChassisSpeeds(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                    vx, 
                    vy,
                    w, 
                    Rotation2d.fromDegrees(localizer.getPose().getRotation().getDegrees()) // gets fused heading
                )
            );
    }
    
    // Allow driver to zero the drive subsystem heading for field-centric control.
    if(m_OI.getDriverMenuButton())
    {
      drivetrain.zeroHeading();
    }

    if(m_OI.getDriverAButton()){
      Rotation2d zeroRotate = new Rotation2d();
      Pose2d zero = new Pose2d(0.0, 0.0, zeroRotate);
      drivetrain.resetOdometry(zero);
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
