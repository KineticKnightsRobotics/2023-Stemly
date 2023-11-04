// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.lang.System;
import frc.robot.RobotContainer;
import frc.robot.subsystems.sub_Drive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class drv_GyroBalance extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final sub_Drive m_subsystem;
  private double direction_multi, desired_deg, diff_deg, cur_deg, previous_deg,speed;
  private float timer;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public drv_GyroBalance(sub_Drive subsystem) {
    m_subsystem = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // m_subsystem.calibrationPitch = m_subsystem.gyro.getRoll();
    desired_deg = -1;  // We want the gyro to be at 0 degrees 
  }
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putBoolean("OnSlope()", m_subsystem.OnSlope());
    SmartDashboard.putNumber("deff deg", diff_deg);
    // Command Functions only opperate once the Gyro reads that the robot is on the slope
    
    if (m_subsystem.OnSlope()) {
      // Balancing - Front End 
      if (m_subsystem.ObtainPitch() > 0) {
        diff_deg = m_subsystem.ObtainPitch() - desired_deg;
        // Determing Speed
        if (m_subsystem.ObtainPitch() > desired_deg + 14.3) {
          m_subsystem.ArcadeDrive(-0.11, 0);
        }
        else if (m_subsystem.ObtainPitch() > desired_deg + 13){
          m_subsystem.ArcadeDrive(-0.06, 0);
        }
        else {
          m_subsystem.ArcadeDrive(0, 0);
        }
      // Balancing - Back End
      } else {  
        diff_deg = desired_deg - m_subsystem.ObtainPitch();
        // Determing Speed
        if (m_subsystem.ObtainPitch() < desired_deg - 13.5) {
          m_subsystem.ArcadeDrive(0.19, 0);
        }
        else {
          m_subsystem.ArcadeDrive(0, 0);
        }
      } 
    }
    
    
  /* 
    float cur_time = System.nanoTime();
    cur_deg = m_subsystem.ObtainPitch();
    speed = -cur_deg;
    if (speed > 1) {speed = 1;}
    if (speed < -1){speed =-1;}

    SmartDashboard.putBoolean("Balanced?", m_subsystem.OnSlope() == true);
    SmartDashboard.putNumber("Current Degree", cur_deg);
    SmartDashboard.putNumber("Previous Degree", previous_deg);
    String state = "Idle";
    SmartDashboard.putNumber("Delta Angle", Math.abs(cur_deg - previous_deg));
    SmartDashboard.putNumber("speed", speed);
    if (m_subsystem.OnSlope()) {
        //First step; If the robot is just getting on the scale, needs to drive faster to make it up.
        
        if (Math.abs(cur_deg) > 5 && Math.abs(cur_deg - previous_deg) < 0.3
        ) {
          state = "First Step";
          //limit of 30% speed
          m_subsystem.ArcadeDrive(0.08 * speed, 0.0);
        }
        
        //Second step; The robot will begin to slow down
        else if (Math.abs(cur_deg) < 5 && Math.abs(cur_deg) > desired_deg + 2) {
          state = "Second Step";
          //limit of 5% speed.
          m_subsystem.ArcadeDrive(0.02 * speed, 0.0);
        }
        else if (Math.abs(cur_deg) > desired_deg + 0.50 && Math.abs(cur_deg) < desired_deg + 2) {
          state = "Third Step";
          //Third step; The robot will crawl forwards
          m_subsystem.ArcadeDrive(0.012 * speed, 0.0);
        }
        
        else {m_subsystem.ArcadeDrive(0.0, 0.0); state = "Falling";}
      }
      else {m_subsystem.ArcadeDrive(0.0, 0.0); state = "Idle";}
    
    float elapsed = System.nanoTime() - cur_time;
    timer += elapsed;
    SmartDashboard.putNumber("MS since prev Deg", timer);
    SmartDashboard.putNumber("elapsed", elapsed);

    if (timer >= 75) {
      previous_deg = cur_deg;
      timer = 0;
    }
    SmartDashboard.putString("Stage", state);
    */
  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
