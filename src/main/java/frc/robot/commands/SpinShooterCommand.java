package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.IntakeShooterSubsystem;


public class SpinShooterCommand extends Command{
    private final IntakeShooterSubsystem m_subsystem;

    public SpinShooterCommand(IntakeShooterSubsystem subsystem) {
        m_subsystem = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(subsystem);

    }

    @Override
  public void initialize() {
    
    // Called when the command is initially scheduled.
  }

  @Override
  public void execute() {
    m_subsystem.ShooterMotorStartSpin();
    // Called every time Command is scheduled
  }

  @Override
  public void end(boolean interrupted) {
    //Called when command ends or is interrupted
  }

  @Override
  public boolean isFinished() {
    m_subsystem.ShooterMotorStopSpin();
    //Called when Command is finished
    return false;
  }
    


    
}
