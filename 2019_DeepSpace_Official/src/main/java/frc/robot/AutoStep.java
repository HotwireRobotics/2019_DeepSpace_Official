package frc.robot;



import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.Ultrasonic;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Timer;

public class AutoStep {
	public enum StepType {
		Rotate, RotateRight, Forward, UltrasonicTarget, AlignUltrasonicLeft, LeftTurnSide, NavxReset, Push, RightTurnSide, WallTrackLeft, WallTrackRight, ShootInSwitch, Backup, Straighten, TimedForward, RobotTurn, Wait, ForwardPickup, ShootSwitch, StraightenLeft
	}

	public StepType type;
	public DriveTrain drivetrain;
	public AHRS navx;
	public float rotateTarget;
	public float speed;
	public int left;
	public boolean isDone;
	public Encoder encoder;
	public float encoderTarget;
	public float ultrasonicTarget;
	public Timer navxTime;
	public Robot robot;
	public AutoStep(DriveTrain choochoo, AHRS gyro, Ultrasonic lsonar, Robot robot) {
		drivetrain = choochoo;
		navx = gyro;
	
		this.robot = robot;

		isDone = false;
	}

	public void MoveForward(float encode, float sped) {
		type = StepType.Forward;
		encoderTarget = encode;
		speed = sped;
	}
	public void RotateRight(float deg, float sped){
		rotateTarget = deg;
		type = StepType.RotateRight;
		speed = sped;
	}
	public void Rotate(float degL, float degR, float sped, int leftorright){
		rotateTarget = -degL;
		
		type = StepType.Rotate;
		speed = sped;
		left = leftorright;
	}
	public void UltrasonicTarget(float dist, float sped) {
		type = StepType.UltrasonicTarget;
	
		ultrasonicTarget = dist;
		speed = sped;
	}
	public void LeftTurnSide(float deg, float sped){
		type = StepType.LeftTurnSide;
		rotateTarget = deg;
		speed = sped;		
	}
	public void RightTurnSide(float deg, float sped){
		type = StepType.RightTurnSide;
		rotateTarget = deg;
		speed = sped;		
	}
	public void NavxReset(float time){
		type = StepType.NavxReset;

	}
	public void AlignUltrasonicLeft(float dist, float sped) {
		type = StepType.AlignUltrasonicLeft;
		ultrasonicTarget = dist;
		speed = sped;
	}
	public void Push(float time, float sped){
		type = StepType.Push; 
		speed = sped;

	}
	public void WallTrackRight(float sped){
		type = StepType.WallTrackRight;
		this.speed = sped;
	}
	public void WallTrackLeft(float sped){
		this.speed = sped;
		type = StepType.WallTrackLeft;
	}
	public void ShootInSwitch(){

		type = StepType.ShootInSwitch;
	}
	public void Backup(float sped, float time){
		this.speed = sped;
	
		type = StepType.Backup;
	}
	public void Straighten(float sped, float deg, float deg2){
		this.speed = sped;
		rotateTarget = deg;

		type = StepType.Straighten;
	}

	public void TimedForward(float sped, float time){
		this.speed = sped;

		type = StepType.TimedForward;

	}
	public void RobotTurn(float sped, float degLeft, float degRight, float revLeft, float revRight){
		this.speed = sped;
	
		type = StepType.RobotTurn;

	}
	public void Wait(float time){
	
		type = StepType.Wait;
	}
	public void ForwardPickup(float sped, float time, float time2){
		
		this.speed = sped;
		type = StepType.ForwardPickup;
	}
	public void ShootSwitch(float speedLeft, float speedRight, float time){

		type = StepType.ShootSwitch;
	}
	public void StraightenLeft(){
		type = StepType.StraightenLeft;
	}

	public void Update() {

		drivetrain.SetLeftSpeed(0.0f);
		drivetrain.SetRightSpeed(0.0f);
		double adjustment = Math.pow(35.0f, speed);
		if (type == StepType.Rotate) {
			if (DriverStation.getInstance().getGameSpecificMessage().charAt(0) == 'R'){
		
			}
			if (navx.getYaw() < rotateTarget - adjustment) {
				drivetrain.SetLeftSpeed(left * speed);
				drivetrain.SetRightSpeed(left * speed);

			} else {
				isDone = true;
			}
		}
		if (type == StepType.RotateRight) { //TODO remove RotateRight
			if (navx.getYaw() > rotateTarget + adjustment) {
				drivetrain.SetLeftSpeed(-speed);
				drivetrain.SetRightSpeed(-speed);
			} else {
				isDone = true;
			}
		}
		if (type == StepType.Forward) {
			if (Math.abs(encoder.getDistance()) < encoderTarget) {
				drivetrain.DriveStraight(speed, false);
			} else {
				isDone = true;
			}
		}
		if (type == StepType.UltrasonicTarget) {
			
		}
		if (type == StepType.AlignUltrasonicLeft) {
			
		}
		if (type == StepType.Backup){
	
		}
		if(type == StepType.Straighten){
			if(DriverStation.getInstance().getGameSpecificMessage().charAt(0) == 'L'){
				if( robot.navx.getYaw() < rotateTarget){
					drivetrain.SetLeftSpeed(-speed);
					drivetrain.SetRightSpeed(-speed);
				}else{
					
					isDone = true;
				}
			}else{
			
			}
		}

		if(type == StepType.LeftTurnSide){
			if((Math.abs(navx.getYaw()) < rotateTarget)){
				drivetrain.SetLeftSpeed(speed);
				
			}else{
				isDone= true;
				
			}
		}
		if(type == StepType.RightTurnSide){
			if((Math.abs(navx.getYaw()) < rotateTarget)){
				drivetrain.SetRightSpeed(-speed);
				
			}else{
				
				isDone= true;
			}
		}
		
		
		
			
		
		if(type == StepType.ForwardPickup){
			
		}
		if(type == StepType.Wait){
		
		}
		if(type == StepType.WallTrackLeft){
		
		}
		if(type == StepType.ShootInSwitch){
			
		}
		if(type == StepType.TimedForward){
			
		}
		if(type == StepType.RobotTurn){
			if(DriverStation.getInstance().getGameSpecificMessage().charAt(0) == 'L'){
				
				
			}else{
				
			
			}
		}
		if(type == StepType.ShootSwitch){
			
		}
	}


	public void InitStep()
	{
	
	}

	public void LogInfo(String info) {
		System.out.println(info + ";    ");
	}

}