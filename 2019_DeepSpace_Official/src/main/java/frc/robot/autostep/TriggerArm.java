package frc.robot.autostep;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.DriveTrain;
import frc.robot.*;

public class TriggerArm extends AutoStep {
    public Robot robot;

    public TriggerArm(Robot robot) {
        super(null);
        this.robot = robot;
    }

    public void Begin() {

    }

    public void Update() {
        robot.autoSetArm = true;
        isDone = true;
    }
}