// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
 

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Counter;

public class LIDARSubsystem extends SubsystemBase {
  /** Creates a new LIDARSubsystem. */
    private Counter m_LIDAR;
  public LIDARSubsystem() {
    
    m_LIDAR = new Counter(0);
    m_LIDAR.setMaxPeriod(1.00);
    m_LIDAR.setSemiPeriodMode(true);
    m_LIDAR.reset();
  }
  final double off = 10;
  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    double dist;
    if (m_LIDAR.get() < 1){
      dist = 0;
    } else {
      dist = (m_LIDAR.getPeriod()*1000000.0/10.0) - off;
      SmartDashboard.putNumber("Distance", dist);
    }
  }

  
}
//Duncan was here