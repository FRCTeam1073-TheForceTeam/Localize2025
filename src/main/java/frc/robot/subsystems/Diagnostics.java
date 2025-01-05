// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

/**
 * Base class for supporting diagnostics functionality. You should derive your
 * subsystems from DiagnosticsSubsystem and sub-components can derive from
 * this class to make diagnostics easier.
 */
public interface Diagnostics {
  

  /**
   * Derived classes @override this method to update your diagnostics results.
   * Warning: You cannot expect to call this method from normal periodic code.
   * It may take a very long time to run.
   */
  public boolean updateDiagnostics();

  /**
   * Derived classes @override this method to update diagnostics feedback.
   * @return
   */
  public boolean diagnosticsOk();

  /**
   * Derived classes @override this method to provide diagnostics details
   * @return
   */
  public String getDiagnosticsDetails();

}
