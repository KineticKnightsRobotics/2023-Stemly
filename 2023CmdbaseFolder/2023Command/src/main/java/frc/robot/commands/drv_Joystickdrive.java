// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.GearRatio;
import frc.robot.subsystems.sub_Drive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class drv_Joystickdrive extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final sub_Drive m_subsystem;
  public static double accel_timer;
  public static double init_vel;
  public static double elseSpeed;
  public static boolean done;
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public drv_Joystickdrive(sub_Drive subsystem) {
    m_subsystem = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    accel_timer = 0; 
    elseSpeed = 0;
    done = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    /*
    if (accel_timer == Math.pow(1,9)){
      accel_timer = 0;
      init_vel = ((m_subsystem.leftMotorEncoder.getVelocity()/60) / Constants.GearRatio.lowDrive) * (Math.PI*8)/39.37;

      if (m_subsystem.findAccel(init_vel, Constants.GearRatio.lowDrive)){
        done = true;// if the elseSpeed is finalized it stops updating it by having done 
      }else if(m_subsystem.findAccel(init_vel, Constants.GearRatio.lowDrive) == false){// if max accel is not reached then it updates the else speed
        elseSpeed = RobotContainer.DriveY();
        done = false;
      }
    }
    accel_timer += 1;
    */
    
    if (RobotContainer.drLB()) 
    {m_subsystem.ShiftHighGear();}
    else
    {m_subsystem.ShiftLowGear();}
    
    double speed = (RobotContainer.DriveY() * 0.65) + sub_Drive.shifterspeed;
    double steer = (RobotContainer.DriveX() * 0.75) + sub_Drive.shifterspeed;  
    //m_subsystem.findAccel(init_vel, Constants.GearRatio.lowDrive);

    /* 
    double speedmod = 1.0;
    SmartDashboard.putBoolean("Slow Mode", RobotContainer.drvstick.getRawButton(Constants.drA));
    if (RobotContainer.drvstick.getRawButton(Constants.drB)) {
      speedmod = 0.5;
      speed *= speedmod;
      steer *= speedmod;
    }
    */

    m_subsystem.ArcadeDrive(speed, steer);

  
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
