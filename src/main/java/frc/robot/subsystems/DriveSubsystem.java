// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;

//import java.util.function.Supplier;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;
//import com.studica.frc.AHRS.NavXUpdateRate;

import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class DriveSubsystem extends SubsystemBase {

    private SwerveModuleCanCoder m_frontLeft;
    private SwerveModuleCanCoder m_frontRight;
    private SwerveModuleCanCoder m_backLeft;
    private SwerveModuleCanCoder m_backRight;
    private AHRS m_navX;

    private boolean m_initalized = false;

    private double m_Goffset = 0.0;

    public static final double kMaxSpeedMetersPerSecond = 4;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;// kMaxSpeedMetersPerSecond /
                                                                          // Math.hypot(0.381, 0.381);
    private static final double kHalfTrackWidthMeters = 0.5715 / 2.0;
    private final Translation2d kFrontLeftLocation = new Translation2d(kHalfTrackWidthMeters, kHalfTrackWidthMeters);
    private final Translation2d kFrontRightLocation = new Translation2d(kHalfTrackWidthMeters, -kHalfTrackWidthMeters);
    private final Translation2d kBackLeftLocation = new Translation2d(-kHalfTrackWidthMeters, kHalfTrackWidthMeters);
    private final Translation2d kBackRightLocation = new Translation2d(-kHalfTrackWidthMeters, -kHalfTrackWidthMeters);

    private Field2d m_field = new Field2d();
 private SwerveDriveKinematics m_kinematics;
    private SwerveDriveOdometry m_odometry;

    public int m_ticks = 0;

    /** Creates a new DriveSubsystem. */
    public DriveSubsystem()
    { 

    m_navX = new AHRS(NavXComType.kMXP_SPI);
    m_navX.reset();
    m_frontLeft= new SwerveModuleCanCoder(1, 21,
                135.65, "Front Left");
    m_frontRight= new SwerveModuleCanCoder(2, 22,
                87.95, "Front Right");
     m_backLeft = new SwerveModuleCanCoder(3,23,
                106.85, "Back Left");
    m_backRight = new SwerveModuleCanCoder(4, 24,
                158.25, "Back Right");
    m_kinematics= new SwerveDriveKinematics(kFrontLeftLocation, kFrontRightLocation, kBackLeftLocation,
                kBackRightLocation);
     m_odometry = new SwerveDriveOdometry(m_kinematics, getAngle(), getPositions());

    HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_MaxSwerve);
    
    try{
      var config = RobotConfig.fromGUISettings();
    
     // Configure AutoBuilder last
    AutoBuilder.configure(
            this::getPose, // Robot pose supplier
            this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
            this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            (speeds, feedforwards) -> driveRobotRelative(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
            new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
                    new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                    new PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants
            ),
            config, // The robot configuration
            () -> {
              // Boolean supplier that controls when the path will be mirrored for the red alliance
              // This will flip the path being followed to the red side of the field.
              // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

              var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
              }
              return false;
            },
            this // Reference to this subsystem to set requirements
    );
    } catch (Exception e) {
      // Handle exception as needed
      e.printStackTrace();

    }
        
    }public ChassisSpeeds getRobotRelativeSpeeds()
    {
        return m_kinematics.toChassisSpeeds(
            m_frontLeft.getSwerveModuleState(),
            m_frontRight.getSwerveModuleState(),
            m_backLeft.getSwerveModuleState(),
            m_backRight.getSwerveModuleState());
    }
public void resetPose(Pose2d pose) {
    m_odometry.resetPosition(
        getAngle(),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_backLeft.getPosition(),
            m_backRight.getPosition()
        },
        pose);
  }
    public void initalize() {
        if (m_initalized)
            return;
        resetSteeringMotorsToAbsolute();
        
        /*var m_trajectory= TrajectoryGenerator.generateTrajectory(new Pose2d(0,0, Rotation2d.fromDegrees(0)), 
            List.of(new Translation2d(1,1), new Translation2d(2,-1)),
            new Pose2d(3,0, Rotation2d.fromDegrees(0)),
            new TrajectoryConfig(Units.feetToMeters(3.0), Units.feetToMeters(3.0)));
        */
        //PathPlannerLogging.setLogActivePathCallback((poses));
        PathPlannerLogging.setLogActivePathCallback((poses) -> m_field.getObject("path").setPoses(poses));

        SmartDashboard.putData("Field", m_field);
        //m_field.getObject("traj").setTrajectory(m_trajectory);
    
        m_initalized = true;
    }
    public void zeroHeading() {
    m_Goffset = (m_Goffset + getAngle().getDegrees())%360;
    m_navX.reset();
  }

    private void resetSteeringMotorsToAbsolute()
    {
        m_frontLeft.resetAngleEncoderToAbsolute();
        m_frontRight.resetAngleEncoderToAbsolute();
        m_backLeft.resetAngleEncoderToAbsolute();
        m_backRight.resetAngleEncoderToAbsolute();
    }

    public void smartDashboardUpdate() {
        m_frontLeft.smartDashboardUpdate();
        m_frontRight.smartDashboardUpdate();
        m_backLeft.smartDashboardUpdate();
        m_backRight.smartDashboardUpdate();

        

    }
    @Override
    public void periodic() {
        
        m_odometry.update(getAngle(), getPositions());
        smartDashboardUpdate();
        m_field.setRobotPose(m_odometry.getPoseMeters());
        m_ticks++;
        if (m_ticks % 15 != 7)
            return;

        SmartDashboard.putNumber(getName() + "/Angle", getAngle().getDegrees());
        SmartDashboard.putNumber(getName() + "/Goffset", m_Goffset);
        SmartDashboard.putNumber(getName() + "/Angle + Goffset", getAngle().getDegrees() + m_Goffset);
        SmartDashboard.putNumber("Match Time", DriverStation.getMatchTime());
        
    }
    
    private void driveRobotRelative(ChassisSpeeds chassisSpeeds)
    {
        drive(m_kinematics.toSwerveModuleStates(chassisSpeeds));
    }

    private void drive(SwerveModuleState[] states)
    {
        SwerveDriveKinematics.desaturateWheelSpeeds(states, kMaxSpeedMetersPerSecond);

        m_frontLeft.setDesiredState(states[0]);
        m_frontRight.setDesiredState(states[1]);
        m_backLeft.setDesiredState(states[2]);
        m_backRight.setDesiredState(states[3]);
    }

    public void drive(double xMetersPerSecond, double yMetersPerSecond,
            double rotationRadiansPerSecond, boolean fieldRelative)
    {
        SwerveModuleState[] states = m_kinematics.toSwerveModuleStates(fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(xMetersPerSecond, yMetersPerSecond, rotationRadiansPerSecond,
                        getAngle())
                : new ChassisSpeeds(xMetersPerSecond, yMetersPerSecond, rotationRadiansPerSecond));

        drive(states);
    }

    public SwerveDriveKinematics getKinematics() {
        return m_kinematics;
    }

    public ChassisSpeeds getChassisSpeeds()
    {
        return m_kinematics.toChassisSpeeds(
            m_frontLeft.getSwerveModuleState(),
            m_frontRight.getSwerveModuleState(),
            m_backLeft.getSwerveModuleState(),
            m_backRight.getSwerveModuleState());
    }

    public Pose2d getPose() {
        return m_odometry.getPoseMeters();
    }

    public void setDesiredState(SwerveModuleState[] states) {
        m_frontLeft.setDesiredState(states[0]);
        m_frontRight.setDesiredState(states[1]);
        m_backLeft.setDesiredState(states[2]);
        m_backRight.setDesiredState(states[3]);
    }

    public Rotation2d getAngle() {
        return m_navX.getRotation2d().unaryMinus();
    }

    public void resetOdometry(Pose2d pose) {
        m_odometry.resetPosition(getAngle(), getPositions(), pose);
    }

    private SwerveModulePosition[] getPositions() {
        SwerveModulePosition[] swerveStates = { m_frontLeft.getPosition(), m_frontRight.getPosition(),
                m_backLeft.getPosition(), m_backRight.getPosition() };
        return swerveStates;
    }

    public void stopAndLockWheels() {
        m_frontLeft.lockWheelAtAngleInDegrees(-45);
        m_frontRight.lockWheelAtAngleInDegrees(-45);
        m_backLeft.lockWheelAtAngleInDegrees(-45);
        m_backRight.lockWheelAtAngleInDegrees(-45);
    }

    public void checkRelativeEncoderToAbsoluteEncoder()
    {
        m_frontLeft.checkRelativeEncoderToAbsoluteEncoder();
        m_backLeft.checkRelativeEncoderToAbsoluteEncoder();
        m_frontRight.checkRelativeEncoderToAbsoluteEncoder();
        m_backRight.checkRelativeEncoderToAbsoluteEncoder();
    }

    public void resetNavX() {
        m_navX.reset();
        setStartAngle(0);
        resetSteeringMotorsToAbsolute();
    }

    public Pose2d m_targetDriveToPose2d;
    public void resetPoseFromLimelight()
    {
        // resetOdometry(new Pose2d( ));   // from lime light

        // switch based on april tag # for target position
        int targetAprilTag = 8;
        switch (targetAprilTag) {
            case 8:
                m_targetDriveToPose2d = new Pose2d(1.5, 5.52, Rotation2d.fromDegrees(0));
                break;
        
            default:
                break;
        }
    }

    public void setStartAngle(double angle) {
        m_navX.setAngleAdjustment(angle);
    }
}