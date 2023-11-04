// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.Constants.Scoring;
import frc.robot.subsystems.sub_Arm;

/** An example command that uses an example subsystem. */
public class arm_ManualControl extends CommandBase {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final sub_Arm m_subsystem;
    private static int kickS_timer; 

    /**
     * Creates a new ExampleCommand.
     *
     * @param subsystem The subsystem used by this command.
     */
    public arm_ManualControl(sub_Arm subsystem) {
        m_subsystem = subsystem;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(subsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_subsystem.spinClamp(0.1);
       //m_subsystem.closeSevro();
       kickS_timer = 0;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {


        // Driver Control Arm Movement
        if(RobotContainer.drvstick.getPOV() == 0.0){  // When the Top Dpad button is pressed, it moves the arm up
            m_subsystem.liftArm(-0.75);
        } else if (RobotContainer.drvstick.getPOV() == 180){ // When the Bottom Dpad button is pressed, it moves the arm down
            m_subsystem.liftArm(0.75);
        } else if (RobotContainer.drvstick.getPOV() == 90 && m_subsystem.getExtEnc() < -230){
            m_subsystem.extendMotor(-0.3);//slows down extender while going out
        } else if (RobotContainer.drvstick.getPOV() == 90 ){
            m_subsystem.extendMotor(-0.60);
        }else if (RobotContainer.drvstick.getPOV() == 270){
            m_subsystem.extendMotor(0.60);
        } else {
            m_subsystem.extendMotor(0);
            m_subsystem.liftArm(0);
        }


        if(RobotContainer.drRT() == false && RobotContainer.drLT()== false){
            if (m_subsystem.clampState()) {
                m_subsystem.spinClamp(0.0);
            }
            else {
                m_subsystem.spinClamp(0.05);
            }

        
            
        }else if(RobotContainer.drRT()){
            m_subsystem.spinClamp(-0.08);
        }else if (RobotContainer.drLT()){
            m_subsystem.spinClamp(0.4);
        }

    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {

       //m_subsystem.closeSevro();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}