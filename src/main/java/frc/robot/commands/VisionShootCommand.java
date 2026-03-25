package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.JetsonConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.JetsonSubsystem;

public class VisionShootCommand  extends Command {
    private int m_ticks;
    private double m_targetShooterSpeed;
    private  ShooterSubsystem m_shooter;
    private  TurretSubsystem m_turret;
     private  PIDController m_turretPID;
     private  JetsonSubsystem m_jetson;
     private DriveSubsystem m_drive;

    public VisionShootCommand(JetsonSubsystem jetson,ShooterSubsystem shooter, TurretSubsystem turret, DriveSubsystem drive) {
        m_shooter=shooter;
        m_turret=turret;
        m_jetson=jetson;
        m_drive=drive;
        m_turretPID = new PIDController(0.02, 0.0, 0.001);
        m_turretPID.setTolerance(2.0); // 2 degree tolerance
        m_turretPID.enableContinuousInput(-180, 180);
    }
    @Override
    public void initialize() {
        m_ticks = 0;
        m_targetShooterSpeed = 0.0;
        //m_targetAnglerSpeed = 0.0;
        //m_turretPID.reset();
        //m_turretPID.setSetpoint(0.0); // Target: 0 degrees error
    }
     @Override
    public void execute() {
        Pose2d robotPose = m_drive.getPose();
        double[] hubPos = m_jetson.getHubPosition();

        // Calculate distance from robot to HUB
        double dx = hubPos[0] - robotPose.getX();
        double dy = hubPos[1] - robotPose.getY();
        double distanceToHub = Math.sqrt(dx * dx + dy * dy);
        
        SmartDashboard.putNumber("VisionShoot/Distance", distanceToHub);
    }
}
