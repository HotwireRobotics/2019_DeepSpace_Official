package frc.robot.autostep;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.interfaces.Potentiometer;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.DriveTrain;
import frc.robot.*;


public class LimelightTrack extends AutoStep {

    public Robot robot;
    public Robot.LimelightPlacement placement;

    public LimelightTrack(DriveTrain driveTrain, Robot robot, Robot.LimelightPlacement placement) {
        super(driveTrain);
        this.robot = robot;
        this.placement = placement;
    }

    public void Begin() {
    }

    public void Update() {
        robot.Limelight(placement);
    }
}