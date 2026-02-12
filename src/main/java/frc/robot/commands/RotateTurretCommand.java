package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.TurretSubsystem;
public class RotateTurretCommand extends Command {
   private TurretSubsystem m_turret;
   private Supplier<Double> m_speed = () -> 0.0;
   private Supplier<Double> m_rightX;
   public RotateTurretCommand(TurretSubsystem Turret, Supplier<Double> rightX){
    addRequirements(Turret);
    m_turret = Turret;
    m_rightX=rightX;
    
   }
    //only runs once
    @Override
    public void initialize() {
        m_turret.RotateTurret(m_rightX);
    }
//everyt time it is ran
    @Override
  public void execute() {
    m_turret.RotateTurret(m_rightX);
  }
   @Override
  public void end(boolean interrupted) {
    m_turret.RotateTurret(()->0.0);
}
   @Override
  public boolean isFinished() {
    return false;
  }
}
