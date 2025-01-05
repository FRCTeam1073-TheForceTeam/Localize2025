// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/* The diagnostics base for subsystems */
public class DiagnosticsSubsystem extends SubsystemBase implements Diagnostics {
  private boolean diagnosticsOk = true;
  private String diagnosticsDetails = "";

  /** Creates a new Diagnostics. */
  public DiagnosticsSubsystem() {
  
  }

  @Override
  public boolean updateDiagnostics() { 
    return false;
  }

  @Override 
  public boolean diagnosticsOk() {
    return this.diagnosticsOk;
  }

  @Override
  public String getDiagnosticsDetails(){
    return this.diagnosticsDetails;
  }

  /** This method should only be run from the subsystem's updateDiagnostics method! */
  public boolean setDiagnosticsFeedback(String diagnosticDetails, boolean ok){
    // this.diagnosticsDetails = diagnosticDetails;
    // this.diagnosticsOk = ok;
    // return this.diagnosticsOk;
    return false;
  }

  /** Run this method in RobotContainer's printAllFalseDiagnostics method to print error messages to the console. */
  public boolean printDiagnostics(boolean disabled){
    // if(!this.diagnosticsOk && disabled){
    //   DriverStation.reportError(this.diagnosticsDetails, false);
    // }
    return false;
  }
}
