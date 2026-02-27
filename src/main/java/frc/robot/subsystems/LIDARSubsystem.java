// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
 

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.Timer;

public class LIDARSubsystem extends SubsystemBase {
  /** Creates a new LIDARSubsystem. */
  private Counter m_RAMP_LIDAR;
  private Counter m_TANK_LIDAR;
  double ramp_dist;
  double tank_dist;
  double empty_time;

  public LIDARSubsystem() {
    
    m_RAMP_LIDAR = new Counter(Constants.lidarConstants.rampCounterID);
    m_RAMP_LIDAR.setMaxPeriod(1.00);
    m_RAMP_LIDAR.setSemiPeriodMode(true);
    m_RAMP_LIDAR.reset();
    
    m_TANK_LIDAR = new Counter(Constants.lidarConstants.tankCounterID);
    m_TANK_LIDAR.setMaxPeriod(1.00);
    m_TANK_LIDAR.setSemiPeriodMode(true);
    m_TANK_LIDAR.reset();
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    if (m_RAMP_LIDAR.get() < 1){
      ramp_dist = 0;
    } else {
      ramp_dist = (m_RAMP_LIDAR.getPeriod()*1000000.0/10.0) - Constants.lidarConstants.LIDAROffset;
      SmartDashboard.putNumber("Distance", ramp_dist);
    }

    if(ramp_dist < Constants.lidarConstants.rampFullDistance) {
      empty_time = Timer.getFPGATimestamp();
    }

    if (m_TANK_LIDAR.get() < 1){
      tank_dist = 0;
    } else {
      tank_dist = (m_TANK_LIDAR.getPeriod()*1000000.0/10.0) - Constants.lidarConstants.LIDAROffset;
      SmartDashboard.putNumber("Distance", tank_dist);
    }

    SmartDashboard.putBoolean("indexer_full", indexer_full());
    SmartDashboard.putNumber("tank_full", get_tank_percentFull());
    
    
  }

  public boolean indexer_full() {
    return (Timer.getFPGATimestamp() - empty_time) < Constants.lidarConstants.rampEmptyTime;
  }

  public double get_tank_percentFull() {
    return 1 - (tank_dist / Constants.lidarConstants.tankFullDistance); 
  }

  public boolean tank_full() {
    return tank_dist<Constants.lidarConstants.tankFilledDistance;
  }
  
}
