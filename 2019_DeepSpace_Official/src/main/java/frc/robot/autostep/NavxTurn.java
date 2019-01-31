package frc.robot.autostep;

import com.kauailabs.navx.frc.AHRS;
import frc.robot.DriveTrain;

public class NavxTurn extends AutoStep {

    public AHRS navx;
    public float turnDegree;
    public float speed;

    public NavxTurn(DriveTrain driveTrain, AHRS navx, float turnDegree, float speed) {
        super(driveTrain);
        this.navx = navx;
    }

    public void Begin() {
        driveTrain.SetLeftSpeed(speed);
        driveTrain.SetRightSpeed(-speed);
    }

    public void Update() {
        if (navx.getYaw() > turnDegree) {
            isDone = true;
        }
    }
}