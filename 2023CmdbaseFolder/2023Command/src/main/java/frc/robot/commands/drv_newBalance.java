// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.sub_Drive;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class drv_newBalance extends CommandBase {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final sub_Drive m_subsystem;
    private double init_pitch;
    private double current_pitch;
    private boolean Attemptbalance;
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public drv_newBalance(sub_Drive subsystem) {
    m_subsystem = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    init_pitch = m_subsystem.ObtainPitch();
    Attemptbalance = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    current_pitch = m_subsystem.ObtainPitch() - init_pitch;
    if (current_pitch < 1){ // if robot is hasnt gone over ramp
        m_subsystem.ArcadeDrive(-0.08, 0);
    }else if (current_pitch < 0.3 && current_pitch > -0.3){
        m_subsystem.ArcadeDrive(0, 0);

    }
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
