package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj.PWMSpeedController;

public class AutonomousCommand extends CommandBase{

    private final DriveSubsystem m_subsystem;

 /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public AutonomousCommand(DriveSubsystem subsystem) {
    m_subsystem = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  @Override
  public void initialize() {
    m_subsystem.resetDistance();
  }

  @Override
  public void execute() {
    m_subsystem.arcadeDrive(0.75, 0.0);
  }

  @Override
  public boolean isFinished(){
    double distance = (m_subsystem.getDistance()/8192.0)*8.0*Math.PI;
    if (distance>=120.0){
        return true;
    }
    else {
        return false;
    }
  }

} 