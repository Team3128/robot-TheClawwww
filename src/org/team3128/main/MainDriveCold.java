package org.team3128.main;

import org.team3128.common.drive.TankDrive;
import org.team3128.common.hardware.encoder.velocity.QuadratureEncoderLink;
import org.team3128.common.hardware.lights.LightsColor;
import org.team3128.common.hardware.lights.LightsSequence;
import org.team3128.common.hardware.lights.PWMLights;
import org.team3128.common.hardware.motor.MotorGroup;
import org.team3128.common.listener.ListenerManager;
import org.team3128.common.listener.controllers.ControllerXbox;
import org.team3128.common.multibot.MainClass;
import org.team3128.common.multibot.RobotTemplate;
import org.team3128.common.util.GenericSendableChooser;
import org.team3128.common.util.Log;
import org.team3128.common.util.units.Length;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Main class for our 2015 robot, The Clawwww minus the Claw (www) ( just drive ).
 * @author Jamie (modified by Wesley)
 *
 */
public class MainDriveCold extends MainClass
{
	
	public ListenerManager lmXbox;
	
	public MotorGroup _pidTestMotor;
	
	public MotorGroup leftMotors;
	public MotorGroup rightMotors;
	public QuadratureEncoderLink leftDriveEncoder;
	public QuadratureEncoderLink rightDriveEncoder;
	public PowerDistributionPanel powerDistPanel;
	
	public TankDrive drive;
	
	int cameraHandle;
	PWMLights lights;
	
	LightsSequence lightShowSequence;
	
	
	public MainDriveCold()
	{	
		lmXbox = new ListenerManager(new Joystick(0));
		powerDistPanel = new PowerDistributionPanel();
		
		leftDriveEncoder = new QuadratureEncoderLink(0,	1, 128, false);
		rightDriveEncoder = new QuadratureEncoderLink(3, 4, 128, true);
		
		leftMotors = new MotorGroup();
		leftMotors.addMotor(new Talon(0));
		leftMotors.addMotor(new Talon(1));
		
		
		rightMotors = new MotorGroup();
		rightMotors.addMotor(new Talon(2));
		rightMotors.addMotor(new Talon(3));
		rightMotors.invert();
	
		drive = new TankDrive(leftMotors, rightMotors, leftDriveEncoder, rightDriveEncoder, 6 * Length.in * Math.PI, 1, 24.5 * Length.in);
		

				
		lights = new PWMLights(10, 11, 12);
		
		lightShowSequence = new LightsSequence();

		lightShowSequence.addStep(new LightsSequence.Step(LightsColor.new8Bit(0xff, 1, 1), 500, false));
		lightShowSequence.addStep(new LightsSequence.Step(LightsColor.new8Bit(0xff, 0xff, 1), 500, false));
		lightShowSequence.addStep(new LightsSequence.Step(LightsColor.new8Bit(1, 0xff, 1), 500, false));
		lightShowSequence.addStep(new LightsSequence.Step(LightsColor.new8Bit(1, 0xff, 0xff), 500, false));
		lightShowSequence.addStep(new LightsSequence.Step(LightsColor.new8Bit(1, 1, 0xff), 500, false));
		lightShowSequence.addStep(new LightsSequence.Step(LightsColor.new8Bit(0xff, 1, 0xff), 500, false));
		
		lightShowSequence.setRepeat(true);
		
		LightsSequence ccaColorsSequence = new LightsSequence();
		ccaColorsSequence.addStep(new LightsSequence.Step(LightsColor.white, 1000, false));
		ccaColorsSequence.addStep(new LightsSequence.Step(LightsColor.red, 1000, false));
		ccaColorsSequence.setRepeat(true);
		
		lights.executeSequence(ccaColorsSequence);


		//Teleop listeners
		//--------------------------------------------------------------------------------------------

        //NIVision.IMAQdxStartAcquisition(cameraHandle);
		
		//-----------------------------------------------------------
		// Drive code, on Logitech Extreme3D joystick
		//-----------------------------------------------------------
		lmXbox.nameControl(ControllerXbox.START, "ClearStickyFaults");
		lmXbox.nameControl(ControllerXbox.RB, "DriveDoubleSpeed");
		
		lmXbox.nameControl(ControllerXbox.JOY2X, "DriveTurn");
		lmXbox.nameControl(ControllerXbox.JOY1Y, "DriveForwardBackward");
		
		lmXbox.addButtonDownListener("ClearStickyFaults", () ->
		{
			powerDistPanel.clearStickyFaults();
		});
		
		
		lmXbox.addMultiListener(() ->
		{
			double joyX = .75 * lmXbox.getAxis("DriveTurn");
			double joyY = lmXbox.getAxis("DriveForwardBackward");			
			drive.arcadeDrive(joyX, joyY, 1, lmXbox.getButton("DriveDoubleSpeed"));
		}, "DriveTurn", "DriveForwardBackward", "DriveDoubleSpeed");
	}

	protected void initializeRobot(RobotTemplate robotTemplate)
	{	
		robotTemplate.addListenerManager(lmXbox);
		
        Log.info("MainDriveCold", "\"Coldbot\"   Activated");
	}

	protected void initializeDisabled()
	{
	}

	protected void initializeAuto()
	{
	}
	
	protected void initializeTeleop()
	{	
	}

	@Override
	protected void addAutoPrograms(GenericSendableChooser<CommandGroup> autoChooser)
	{
	}

	@Override
	protected void updateDashboard()
	{
		SmartDashboard.putNumber("Total Current: ", powerDistPanel.getTotalCurrent());
	}
}
