package org.team3128.main;

import java.util.LinkedList;

import org.team3128.autonomous.commands.CmdVisionGoTowardsCan;
import org.team3128.common.NarwhalRobot;
import org.team3128.common.drive.TankDrive;
import org.team3128.common.hardware.encoder.both.QuadratureEncoder;
import org.team3128.common.hardware.lights.PWMLights;
import org.team3128.common.hardware.motor.MotorGroup;
import org.team3128.common.listener.ListenerManager;
import org.team3128.common.util.GenericSendableChooser;
import org.team3128.common.util.units.Length;
import org.team3128.util.ParticleReport;
import org.team3128.util.RoboVision;

import com.ni.vision.NIVision.Range;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.vision.AxisCamera;

public class MainCameraTest extends NarwhalRobot
{
	AxisCamera camera;
	RoboVision visionProcessor;
	
	public MotorGroup _pidTestMotor;
	
	public MotorGroup leftMotors;
	public MotorGroup rightMotors;
	public QuadratureEncoder leftDriveEncoder;
	public QuadratureEncoder rightDriveEncoder;
	public PowerDistributionPanel powerDistPanel;
	
	public TankDrive drive;
	
	int cameraHandle;
	PWMLights lights;
	
	ListenerManager manager;
	

	@Override
	protected void constructHardware()
	{
		manager = new ListenerManager(new Joystick(0));
		
		powerDistPanel = new PowerDistributionPanel();
		
		leftDriveEncoder = new QuadratureEncoder(0,	1, 128, false);
		rightDriveEncoder = new QuadratureEncoder(3, 4, 128, true);
		
		leftMotors = new MotorGroup();
		leftMotors.addMotor(new Talon(1));
		leftMotors.addMotor(new Talon(2));
		
		
		rightMotors = new MotorGroup();
		rightMotors.addMotor(new Talon(3));
		rightMotors.addMotor(new Talon(4));
		rightMotors.invert();
	
		drive = new TankDrive(leftMotors, rightMotors, leftDriveEncoder, rightDriveEncoder, 6 * Length.in * Math.PI, 1,  20 * Length.in, 15 * Length.in);		
		
		camera = new AxisCamera("10.31.31.21");
		visionProcessor = new RoboVision(camera, .5, true);
		
		//trash can
//		SmartDashboard.putNumber("minH", 105);
//		SmartDashboard.putNumber("maxH", 137);
//		SmartDashboard.putNumber("minS", 5);
//		SmartDashboard.putNumber("maxS", 128);
//		SmartDashboard.putNumber("minV", 0);
//		SmartDashboard.putNumber("maxV", 255);
//		SmartDashboard.putNumber("aspectRatio",(21.9 * Units.in)/(28.8 * Units.in));
//		SmartDashboard.putNumber("rectangularityScore", 100);
		
/*		SmartDashboard.putNumber("minH", 35);
		SmartDashboard.putNumber("maxH", 70);
		SmartDashboard.putNumber("minS", 10);
		SmartDashboard.putNumber("maxS", 150);
		SmartDashboard.putNumber("minV", 128);
		SmartDashboard.putNumber("maxV", 255);*/
		
		SmartDashboard.putNumber("minH", 0);
		SmartDashboard.putNumber("maxH", 255);
		SmartDashboard.putNumber("minS", 0);
		SmartDashboard.putNumber("maxS", 255);
		SmartDashboard.putNumber("minV", 0);
		SmartDashboard.putNumber("maxV", 255);
		
//		SmartDashboard.putNumber("width", value);
		
		
		SmartDashboard.putNumber("aspectRatio", 1);
		SmartDashboard.putNumber("rectangularityScore", 78.5);
		
		addListenerManager(manager);
	}

	@Override
	protected void setupListeners()
	{
		
	}

	@Override
	protected void constructAutoPrograms(GenericSendableChooser<CommandGroup> autoChooser)
	{
		CommandGroup followCanAuto = new CommandGroup();
		followCanAuto.addSequential(new CmdVisionGoTowardsCan(drive, visionProcessor));
		autoChooser.addDefault("Follow Can Auto", followCanAuto);
	}


	@SuppressWarnings("deprecation")
	@Override
	protected void teleopPeriodic()
	{

			LinkedList<ParticleReport> targets = visionProcessor.findSingleTarget(
					new Range(SmartDashboard.getInt("minH"), SmartDashboard.getInt("maxH")), 
	        		new Range(SmartDashboard.getInt("minS"), SmartDashboard.getInt("maxS")),
	        		new Range(SmartDashboard.getInt("minV"), SmartDashboard.getInt("maxV")),
	        		SmartDashboard.getNumber("aspectRatio",(21.9 * Length.in)/(28.8 * Length.in)),
	        		SmartDashboard.getNumber("rectangularityScore"));
			if(!targets.isEmpty())
			{
				
				ParticleReport targetReport = targets.get(0);
				
				SmartDashboard.putNumber("Target distance (in)", targetReport.computeDistanceHorizontal(21.9 * Length.in) /Length.in);
				SmartDashboard.putNumber("target heading angle", targetReport.getHeadingAngleOffset());
			}
			else
			{
				SmartDashboard.putNumber("Target distance (in)", 0);
				SmartDashboard.putNumber("target heading angle", 0);
			}
		
		
		
	}


	@Override
	protected void teleopInit()
	{
		
	}

	@Override
	protected void autonomousInit()
	{
		
	}

}
