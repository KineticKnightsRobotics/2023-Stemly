// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.sub_Drive;


import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class drv_GyroTurnNumDegrees extends CommandBase {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final sub_Drive m_subsystem;
    private double desired_deg, diff_deg, g_steer; 

    
    /**
     * Creates a new ExampleCommand.
     *
     * @param subsystem The subsystem used by this command.
     */
    public drv_GyroTurnNumDegrees(sub_Drive subsystem, double degree) {
        m_subsystem = subsystem;
        desired_deg = degree;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(subsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() { 
        m_subsystem.ZeroYaw();  // Sets Yaw to Zero   
    }
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        diff_deg = desired_deg - m_subsystem.ObtainYaw();  // Finds the Differnce between the desired degree and current degree
        // Divides the Net Degree by 250 to find steer value; Max cap is 1
        if (diff_deg > 0){
        if (diff_deg/100 >= 0.5){
            g_steer = 0.5;
        } else if (diff_deg/100 > 0.15) {
            g_steer = (diff_deg/100); 
        } else {
            g_steer = 0.15;
        }
        }else if (diff_deg < 0){
            if (diff_deg/100 <= 0.5){
                g_steer = -0.5;
            } else if (diff_deg/100 < 0.15) {
                g_steer = -Math.abs((diff_deg/100)); 
            } else {
                g_steer = -0.15;
            }
        }
    
        m_subsystem.ArcadeDrive(0.0, g_steer); 
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_subsystem.ArcadeDrive(0.0, 0.0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return desired_deg + 5 >= Math.ceil(m_subsystem.ObtainYaw()) && desired_deg - 5 <= Math.ceil(m_subsystem.ObtainYaw());
        //return false;
    }
}