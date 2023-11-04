// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.sub_Drive;

import java.io.Console;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class drv_encTurn extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final sub_Drive m_subsystem;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public drv_encTurn(sub_Drive subsystem) {
    m_subsystem = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_subsystem.ZeroEncoders();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // turns required for 360 * encvalue  per rotation of wheel
    double wheelTurnsLeft = 3.028*Constants.GearRatio.lowDrive;
    double wheelTurnsDone = m_subsystem.getDriveEncoder();
    SmartDashboard.putNumber("wheelTurns left", wheelTurnsLeft);
    SmartDashboard.putNumber("wheelTurns done", wheelTurnsDone);

    // if rotations of the wheel left > # of turns done
    if (wheelTurnsLeft > wheelTurnsDone){
    m_subsystem.m_leftMotor.set(0.2);
    m_subsystem.m_rightMotor.set(-0.2);
  }else {
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
