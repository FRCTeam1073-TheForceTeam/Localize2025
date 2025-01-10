// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AprilTagFinder extends SubsystemBase {
  public PhotonCamera frontLeftCam = new PhotonCamera("LeftCamera");
  public PhotonCamera frontRightCam = new PhotonCamera("Arducam_OV9281_USB_Camera");
  List<PhotonTrackedTarget> responseFL;
  List<PhotonTrackedTarget> responseFR;

  public int wait_counter = 0;

  public List<List<PhotonTrackedTarget>> getCurrentTagData() {
    List<List<PhotonTrackedTarget>> result = new ArrayList<>();
    result.add(responseFL);
    result.add(responseFR);
    return result;
  }

  public void readTagData() {
    responseFL = frontLeftCam.getLatestResult().getTargets();
    responseFR = frontRightCam.getLatestResult().getTargets();
  }

  @Override
  public void periodic() { 
    readTagData();

    if(responseFL.size() > 0) {
      SmartDashboard.putNumber("FL ID", responseFL.get(0).getFiducialId());
    }
    else {
      SmartDashboard.putNumber("FL ID", -1);
    }
    if(responseFR.size() > 0) {
      SmartDashboard.putNumber("FR ID", responseFR.get(0).getFiducialId());
    }
    else {
      SmartDashboard.putNumber("FR ID", -1);
    }
    SmartDashboard.putNumber("Total Tags Seen", responseFL.size() + responseFR.size());
  }
}