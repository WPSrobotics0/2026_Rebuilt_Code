package frc.robot.subsystems;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkRelativeEncoder;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.ResetMode;

import java.util.function.Supplier;

//import com.andymark.jni.AM_CAN_Color_Sensor;
//import com.andymark.jni.AM_CAN_HexBoreEncoder;
//import com.andymark.jni.AM_CAN_HexBoreEncoder.AM_EncoderStatus;
//import com.andymark.jni.AM_CAN_HexBoreEncoder.AM_Encoder_Telemetry;
import com.revrobotics.PersistMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SubsystemConstants;

public class TurretSubsystem  extends SubsystemBase {

    private final SparkMax m_TurretMotor = new SparkMax(SubsystemConstants.kTurretId, MotorType.kBrushless);
    //private final AM_CAN_HexBoreEncoder absEncoder = new AM_CAN_HexBoreEncoder(0);
    public SparkRelativeEncoder m_TurretEncoder = (SparkRelativeEncoder) m_TurretMotor.getEncoder();
    //private AM_Encoder_Telemetry teleData = absEncoder.getTelemetry();
    //private AM_EncoderStatus statData = absEncoder.getStatus();
    public TurretSubsystem() {
        SparkMaxConfig liftConfig =new SparkMaxConfig();
        m_TurretMotor.configure(liftConfig, ResetMode.kResetSafeParameters,PersistMode.kPersistParameters);
    //absEncoder.resetReportPeriod();
    //absEncoder.setZeroHere();

    }
    @Override
    public void periodic() {
        
    }
    public void RotateTurret(Supplier<Double> speed) {
        m_TurretMotor.set(speed.get()/5.0);
        SmartDashboard.putNumber("Spindexer Speed", speed.get());
        //double degrees = absEncoder.getAngleDegrees();
        //double degreesPerSec =  absEncoder.getVelocityDegPerSec();
        //SmartDashboard.putNumber("hex bore angleDegrees", degrees);
        //SmartDashboard.putNumber("hexBoreVelocity", degreesPerSec);

    }
}
