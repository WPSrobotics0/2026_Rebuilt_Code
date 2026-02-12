package frc.robot.subsystems;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.ResetMode;

import java.util.function.Supplier;

import com.revrobotics.PersistMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SubsystemConstants;

public class SpindexerSubsystem  extends SubsystemBase {

    private final SparkMax m_SpindexerMotor = new SparkMax(SubsystemConstants.kSpindexerId, MotorType.kBrushless);
    public SpindexerSubsystem() {
        SparkMaxConfig liftConfig =new SparkMaxConfig();
        m_SpindexerMotor.configure(liftConfig, ResetMode.kResetSafeParameters,PersistMode.kPersistParameters);

    }
    @Override
    public void periodic() {
        
    }
    public void Spin(double speed) {
        m_SpindexerMotor.set(speed);
        SmartDashboard.putNumber("Spindexer Speed", speed);
    }
}
