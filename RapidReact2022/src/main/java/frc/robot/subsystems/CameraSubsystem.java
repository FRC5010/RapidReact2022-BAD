// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoSink;
import edu.wpi.first.cscore.VideoSource.ConnectionStrategy;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CameraSubsystem extends SubsystemBase {
  UsbCamera camera1;
  UsbCamera camera2;
  NetworkTableEntry cameraSelection;
  VideoSink server;

  Joystick operator;
  /** Creates a new CameraSubsystem. */
  public CameraSubsystem() {
    

    camera1 = CameraServer.startAutomaticCapture(0);
    //camera2 = CameraServer.startAutomaticCapture(1);

    cameraSelection = NetworkTableInstance.getDefault().getTable("").getEntry("CameraSelection");
    server = CameraServer.getServer();

    camera1.setConnectionStrategy(ConnectionStrategy.kKeepOpen);
    //camera2.setConnectionStrategy(ConnectionStrategy.kKeepOpen);
    server.setSource(camera1);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

}
