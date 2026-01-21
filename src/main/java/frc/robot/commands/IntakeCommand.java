package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
public class IntakeCommand extends Command {
   private IntakeSubsystem m_intake;
   private double m_speed=1.0;
   public IntakeCommand(IntakeSubsystem intake){
    addRequirements(intake);
    m_intake = intake;
    
   }
    //only runs once
    @Override
    public void initialize() {
        m_intake.setIntakeSpeed(()->m_speed);
    }
//everyt time it is ran
    @Override
  public void execute() {
    m_intake.setIntakeSpeed(()->m_speed);
  }
   @Override
  public void end(boolean interrupted) {
    m_intake.setIntakeSpeed(()->0.0);
}
   @Override
  public boolean isFinished() {
    return false;
  }
}
