package org.team3128.autonomous.programs;

import org.team3128.common.autonomous.movement.CmdTurnGyro;
import org.team3128.common.drive.TankDrive;
import org.team3128.common.util.datatypes.PIDConstants;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.interfaces.Gyro;

public class TestGyroTurnAuto extends CommandGroup
{
    public TestGyroTurnAuto(TankDrive drive, Gyro gyro)
    {
    	addSequential(new CmdTurnGyro(gyro, drive, 90, .5, new PIDConstants(.1, 0, 0), 10000)); //.01 .008 .0001
    			

    }
}
