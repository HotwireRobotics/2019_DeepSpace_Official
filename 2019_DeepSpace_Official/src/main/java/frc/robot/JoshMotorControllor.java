package frc.robot;



import edu.wpi.first.wpilibj.VictorSP;
import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

public class JoshMotorControllor {
	
	public VictorSP victor;
	public TalonSRX talon;
	public float accelValue;
	public float target;
	public boolean usingVictor;
	
	public JoshMotorControllor(int motorpwm, float AcelerationMax, boolean usingVictor)
	{
		this.usingVictor = usingVictor;
		if (usingVictor) {
			victor = new VictorSP(motorpwm);
		} else {
			talon = new WPI_TalonSRX(motorpwm);
		}
		
		accelValue = AcelerationMax;
	}
		
	public void UpdateMotor()
	{
		if (victor != null || talon != null) {
			double curr = 0;
			if(usingVictor) {
				curr = victor.get();
			} else {	
				curr = talon.getMotorOutputPercent();
			}
			
			float newValue = Lerp((float)curr,target,accelValue);
			
			float epsilon = 0.001f;
			if (newValue < epsilon && newValue > -epsilon) 
			{
				newValue = 0.0f;
			}
	
			if (usingVictor) {
				victor.set(newValue);
			} else {
				talon.set(ControlMode.PercentOutput, newValue);
			}
		}
	}
	
	public float Lerp(float v0, float v1, float t)
	{
		return (v0 + t*(v1-v0));
	}
	
	public void SetBrake(){
		//talon.setNeutralMode(NeutralMode.Brake);
	}
	public void SetCoast(){
		//talon.setNeutralMode(NeutralMode.Coast);
	}
}