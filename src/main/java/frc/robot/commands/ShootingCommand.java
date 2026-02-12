package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShootingSubsystem;
public class ShootingCommand extends Command {
   //private ShootingSubsystem m_shooting;
   //private double m_speed=1.0;
   public ShootingCommand(ShootingSubsystem shooting){
    addRequirements(shooting);
    //m_shooting = shooting;
    
   }
    //only runs once
    @Override
    public void initialize() {
        //m_shooting.setShootingSpeed(()->m_speed);
    }
//everyt time it is ran
    @Override
  public void execute() {
    //m_shooting.setShootingSpeed(()->m_speed);
  }
   @Override
  public void end(boolean interrupted) {
    //m_shooting.Speed(()->0.0);
}
   @Override
  public boolean isFinished() {
    return false;
  }
}
