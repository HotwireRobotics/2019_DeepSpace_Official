package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.interfaces.Potentiometer;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Robot extends TimedRobot {

	// Sensors
	public AHRS navx = new AHRS(SPI.Port.kMXP);
	public Ultrasonic Ultrasonic = new Ultrasonic(0, 1);

	// Drivetrain
	public DriveTrain drivetrain;
	public JoshMotorControllor badMotor = new JoshMotorControllor(5, 0.8f, false);

	// neumatics

	// Joysticks
	public Joystick driver;
	public Joystick operator;
	public Joystick debug;

	// Auto

	public void robotInit() {
		drivetrain = new DriveTrain(9, 10, 7, 5, navx);
	}

	public void disabledPeriodic() {
		// Controllers
		driver = new Joystick(0);
		operator = new Joystick(1);
	}

	public void autonomousInit() {

		drivetrain.SetBreak();
	}

	public void autonomousPeriodic() {

		UpdateMotors();
	}

	public void teleopInit() {
		// Controllers
		debug = new Joystick(3);
		driver = new Joystick(0);
		operator = new Joystick(1);

		// Drivetrain
		drivetrain.SetCoast();
		float lerpSpeed = 0.5f;
		drivetrain.SetLeftSpeed(lerpSpeed);
		drivetrain.SetRightSpeed(lerpSpeed);
	}

	public void teleopPeriodic() {

		driver.setRumble(RumbleType.kLeftRumble, 0);
		drivetrain.SetLeftSpeed(0);
		drivetrain.SetRightSpeed(0);

		NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
		NetworkTableEntry tx = table.getEntry("tx");
		NetworkTableEntry ty = table.getEntry("ty");
		NetworkTableEntry ta = table.getEntry("ta");

		double x = tx.getDouble(0.0);
		double y = ty.getDouble(0.0);
		double area = ta.getDouble(0.0);

		// Driver Controls
		ControllerDrive();

		if (driver.getRawButton(5)) {

			float approachSpeed = 0.2f;
			float turnSpeed = 0.2f;
			float buffer = 2.0f;
			float approachtarget = 4;

			if (x >= buffer) {
				drivetrain.SetLeftSpeed(turnSpeed);
				drivetrain.SetRightSpeed(-turnSpeed);
			} else if (x <= -buffer) {
				drivetrain.SetLeftSpeed(-turnSpeed);
				drivetrain.SetRightSpeed(turnSpeed);
			} else {
				drivetrain.SetLeftSpeed(0f);
				drivetrain.SetRightSpeed(0f);
				driver.setRumble(RumbleType.kLeftRumble, 1);
				if (y < approachtarget) {
					drivetrain.SetLeftSpeed(approachSpeed);
					drivetrain.SetRightSpeed(approachSpeed);
				}
			}
		}

		UpdateMotors();
	}

	public void testInit() {
		// Controllers
		debug = new Joystick(3);
		driver = new Joystick(0);
		operator = new Joystick(1);

		// Drivetrain
		float lerpSpeed = 0.5f;
		drivetrain.SetLeftSpeed(lerpSpeed);
		drivetrain.SetRightSpeed(lerpSpeed);
	}

	public void testPeriodic() {

	}

	public void UpdateMotors() {
		drivetrain.Update();

	}

	public float TranslateController(float input) {
		float deadzone = 0.15f;
		if (input > -deadzone && input < deadzone) {
			input = 0.0f;
		}
		float a = 0.7f;
		float output = (a * input * input * input) + (1 - a) * input;
		return output;
	}

	// custom classes
	public void LogInfo(String info) {
		System.out.println(info + ";    ");
	}

	public void ControllerDrive() {
		float horJoystick = TranslateController((float) driver.getRawAxis(0));
		float verJoystick = TranslateController((float) driver.getRawAxis(5));

		drivetrain.SetRightSpeed(-verJoystick + -horJoystick);
		drivetrain.SetLeftSpeed(-verJoystick + horJoystick);
	}
}