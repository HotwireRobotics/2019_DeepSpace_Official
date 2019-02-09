package frc.robot.autostep;

import edu.wpi.first.wpilibj.Timer;
import com.kauailabs.navx.frc.AHRS;

import frc.robot.DriveTrain;

public class NavxReset extends AutoStep {

    public AHRS navx;
    public float resetTimeLengthSeconds;

    public NavxReset(DriveTrain driveTrain, AHRS navx) {
        super(driveTrain);
        this.navx = navx;
    }

    public void Begin() {
        navx.reset();
    }

    public void Update() {
        if (navx.getYaw() == 0.0) {
            isDone = true;
        }
    }
}