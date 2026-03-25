package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.JetsonConstants;

/**
 * Subsystem that reads Jetson AGX Orin vision data from NetworkTables.
 *
 * The Jetson publishes:
 * - FUEL (ball) detections: count, nearest angle/distance, all detections
 * - Robot pose from multi-tag AprilTag estimation (via PhotonVision)
 * - Connection heartbeat
 *
 * This subsystem also tracks ALLIANCE SHIFT timing for strategic decisions.
 */
public class JetsonSubsystem extends SubsystemBase {

    // NetworkTables subscribers for Jetson data
    private final BooleanSubscriber m_connectedSub;
    private final DoubleSubscriber m_fpsSub;

    // FUEL detection subscribers
    private final IntegerSubscriber m_fuelCountSub;
    private final DoubleSubscriber m_fuelNearestAngleSub;
    private final DoubleSubscriber m_fuelNearestDistSub;
    private final DoubleArraySubscriber m_fuelDetectionsSub;

    // Pose estimation subscribers
    private final DoubleSubscriber m_poseXSub;
    private final DoubleSubscriber m_poseYSub;
    private final DoubleSubscriber m_poseHeadingSub;
    private final DoubleSubscriber m_poseTimestampSub;
    private final IntegerSubscriber m_poseTagCountSub;
    private final DoubleSubscriber m_poseAmbiguitySub;

    // Cached values (updated in periodic())
    private boolean m_connected = false;
    private double m_fps = 0.0;
    private int m_fuelCount = 0;
    private double m_nearestFuelAngleDeg = 0.0;
    private double m_nearestFuelDistMeters = 0.0;
    private double[] m_fuelDetections = new double[0];
    private Pose2d m_visionPose = new Pose2d();
    private double m_visionTimestamp = 0.0;
    private int m_tagCount = 0;
    private double m_poseAmbiguity = 1.0;

    // ALLIANCE SHIFT tracking
    private double m_teleopStartTime = -1.0;
    private boolean m_isRedAlliance = false;

    public JetsonSubsystem() {
        NetworkTableInstance ntInst = NetworkTableInstance.getDefault();

        // Jetson main table
        NetworkTable jetsonTable = ntInst.getTable(JetsonConstants.kTableName);
        m_connectedSub = jetsonTable.getBooleanTopic("connected").subscribe(false);
        m_fpsSub = jetsonTable.getDoubleTopic("fps").subscribe(0.0);

        // FUEL detection table
        NetworkTable fuelTable = ntInst.getTable(JetsonConstants.kFuelTableName);
        m_fuelCountSub = fuelTable.getIntegerTopic("count").subscribe(0);
        m_fuelNearestAngleSub = fuelTable.getDoubleTopic("nearest_angle_deg").subscribe(0.0);
        m_fuelNearestDistSub = fuelTable.getDoubleTopic("nearest_dist_m").subscribe(0.0);
        m_fuelDetectionsSub = fuelTable.getDoubleArrayTopic("detections").subscribe(new double[0]);

        // Pose estimation table
        NetworkTable poseTable = ntInst.getTable(JetsonConstants.kPoseTableName);
        m_poseXSub = poseTable.getDoubleTopic("x").subscribe(0.0);
        m_poseYSub = poseTable.getDoubleTopic("y").subscribe(0.0);
        m_poseHeadingSub = poseTable.getDoubleTopic("heading_deg").subscribe(0.0);
        m_poseTimestampSub = poseTable.getDoubleTopic("timestamp").subscribe(0.0);
        m_poseTagCountSub = poseTable.getIntegerTopic("tag_count").subscribe(0);
        m_poseAmbiguitySub = poseTable.getDoubleTopic("ambiguity").subscribe(1.0);
    }

    @Override
    public void periodic() {
        // Read all Jetson data from NetworkTables
        m_connected = m_connectedSub.get();
        m_fps = m_fpsSub.get();

        m_fuelCount = (int) m_fuelCountSub.get();
        m_nearestFuelAngleDeg = m_fuelNearestAngleSub.get();
        m_nearestFuelDistMeters = m_fuelNearestDistSub.get();
        m_fuelDetections = m_fuelDetectionsSub.get();

        m_visionPose = new Pose2d(
            m_poseXSub.get(),
            m_poseYSub.get(),
            Rotation2d.fromDegrees(m_poseHeadingSub.get())
        );
        m_visionTimestamp = m_poseTimestampSub.get();
        m_tagCount = (int) m_poseTagCountSub.get();
        m_poseAmbiguity = m_poseAmbiguitySub.get();

        // Determine alliance color
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            m_isRedAlliance = (alliance.get() == DriverStation.Alliance.Red);
        }

        // Telemetry
        SmartDashboard.putBoolean("Jetson/Connected", m_connected);
        SmartDashboard.putNumber("Jetson/FPS", m_fps);
        SmartDashboard.putNumber("Jetson/FuelCount", m_fuelCount);
        SmartDashboard.putNumber("Jetson/NearestFuelAngle", m_nearestFuelAngleDeg);
        SmartDashboard.putNumber("Jetson/NearestFuelDist", m_nearestFuelDistMeters);
        SmartDashboard.putNumber("Jetson/TagCount", m_tagCount);
        SmartDashboard.putBoolean("Jetson/HubActive", isHubActive());
        SmartDashboard.putNumber("Jetson/ShiftTimeRemaining", getTimeUntilNextShift());
    }

    // ---- FUEL Detection Accessors ----

    /** Whether the Jetson is connected and publishing data. */
    public boolean isConnected() {
        return m_connected;
    }

    /** Number of FUEL detected in the current frame. */
    public int getFuelCount() {
        return m_fuelCount;
    }

    /** Horizontal angle to nearest FUEL in degrees (positive = right). */
    public double getNearestFuelAngle() {
        return m_nearestFuelAngleDeg;
    }

    /** Distance to nearest FUEL in meters. */
    public double getNearestFuelDistance() {
        return m_nearestFuelDistMeters;
    }

    /** Whether any FUEL is currently detected. */
    public boolean hasFuelTarget() {
        return m_connected && m_fuelCount > 0;
    }

    /**
     * All FUEL detections as a flattened array: [angle1, dist1, angle2, dist2, ...]
     */
    public double[] getAllFuelDetections() {
        return m_fuelDetections;
    }

    // ---- Pose Estimation Accessors ----

    /** Robot pose from multi-tag AprilTag estimation. */
    public Pose2d getRobotPose() {
        return m_visionPose;
    }

    /** FPGA timestamp of the vision pose measurement. */
    public double getVisionTimestamp() {
        return m_visionTimestamp;
    }

    /** Number of AprilTags used for the pose estimate. */
    public int getTagCount() {
        return m_tagCount;
    }

    /** Whether the vision pose is trustworthy (enough tags, low ambiguity). */
    public boolean isPoseValid() {
        return m_connected && m_tagCount >= 1 && m_poseAmbiguity < 0.2;
    }

    /** Pose ambiguity (lower = more confident). */
    public double getPoseAmbiguity() {
        return m_poseAmbiguity;
    }

    // ---- ALLIANCE SHIFT Tracking ----

    /** Call this when teleop starts to initialize shift tracking. */
    public void onTeleopInit() {
        m_teleopStartTime = Timer.getFPGATimestamp();
    }

    /**
     * Whether the alliance's HUB is currently active (can score points).
     * Both hubs are active during AUTO, TRANSITION SHIFT, and END GAME.
     * During ALLIANCE SHIFTS 1-4, they alternate every 25 seconds.
     */
    public boolean isHubActive() {
        if (m_teleopStartTime < 0) {
            return true; // Before teleop, assume active (AUTO period)
        }

        double elapsed = Timer.getFPGATimestamp() - m_teleopStartTime;

        // TRANSITION SHIFT: 0-10 seconds into teleop, both hubs active
        if (elapsed < JetsonConstants.kTransitionShiftSeconds) {
            return true;
        }

        // END GAME: last 30 seconds (elapsed > 110 seconds into teleop), both hubs active
        double teleopTotal = 140.0; // 2:20 = 140 seconds
        if (elapsed > (teleopTotal - JetsonConstants.kEndGameSeconds)) {
            return true;
        }

        // ALLIANCE SHIFTS: after transition, 4 shifts of 25 seconds each
        double shiftElapsed = elapsed - JetsonConstants.kTransitionShiftSeconds;
        int shiftIndex = (int) (shiftElapsed / JetsonConstants.kShiftPeriodSeconds);

        // The alliance that scored more in AUTO gets inactive first in SHIFT 1.
        // For simplicity, we alternate based on even/odd shift index.
        // The FMS tells us via game data which alliance goes first --
        // this should be read from DriverStation game data in a real implementation.
        boolean evenShift = (shiftIndex % 2 == 0);

        // Convention: if red alliance scored more in AUTO, red is INACTIVE in shift 1 (even).
        // We default to "active on even shifts" for the robot's alliance.
        // TODO: Read the actual FMS game data to determine which alliance is active first.
        return !evenShift;
    }

    /** Seconds until the next ALLIANCE SHIFT transition. */
    public double getTimeUntilNextShift() {
        if (m_teleopStartTime < 0) {
            return 0.0;
        }

        double elapsed = Timer.getFPGATimestamp() - m_teleopStartTime;

        if (elapsed < JetsonConstants.kTransitionShiftSeconds) {
            return JetsonConstants.kTransitionShiftSeconds - elapsed;
        }

        double shiftElapsed = elapsed - JetsonConstants.kTransitionShiftSeconds;
        return JetsonConstants.kShiftPeriodSeconds - (shiftElapsed % JetsonConstants.kShiftPeriodSeconds);
    }

    /** Whether we are in the END GAME period (last 30 seconds). */
    public boolean isEndGame() {
        if (m_teleopStartTime < 0) {
            return false;
        }
        double elapsed = Timer.getFPGATimestamp() - m_teleopStartTime;
        return elapsed > (140.0 - JetsonConstants.kEndGameSeconds);
    }

    /** Whether the robot's alliance is red. */
    public boolean isRedAlliance() {
        return m_isRedAlliance;
    }

    /**
     * Get the field position of this alliance's HUB.
     */
    public double[] getHubPosition() {
        if (m_isRedAlliance) {
            return new double[] {JetsonConstants.kRedHubX, JetsonConstants.kRedHubY};
        } else {
            return new double[] {JetsonConstants.kBlueHubX, JetsonConstants.kBlueHubY};
        }
    }
}
