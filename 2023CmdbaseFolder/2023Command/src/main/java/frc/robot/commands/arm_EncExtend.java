package frc.robot.commands;

import frc.robot.RobotContainer;
import frc.robot.subsystems.sub_Arm;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class arm_EncExtend extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final sub_Arm m_subsystem;
  private static double m_pos;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public arm_EncExtend (sub_Arm subsystem1, double desired_position) {
    m_subsystem = subsystem1;
    m_pos = desired_position;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem1);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //sub_Arm.openSevro();
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double diff_pos = m_pos - RobotContainer.RoundTo(m_subsystem.getExtEnc(), 1);
    // Fastest Movement (CAP OUT AT 50%)
    if (diff_pos/100 >= 0.5){
      m_subsystem.extendMotor(0.5);
    // Changeable Movement 
    } else if (diff_pos/100 > 0.2) {
      m_subsystem.extendMotor(diff_pos/100);
    // Slowest Movement (MINIMUN AT 10%)
    } else {
      m_subsystem.extendMotor(0.2);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //sub_Arm.closeSevro();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (m_pos <= RobotContainer.RoundTo(m_subsystem.getExtEnc(), 1));
  }
}