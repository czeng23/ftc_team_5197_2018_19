package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

public interface FTCModularDrivetrainDrivable extends FTCModularizableSystems {
    //abstract public void driveLinearInches(double power, double distanceInInches);
    //abstract public void driveCurved();
    //abstract public void turn();
    void teleOpTankDrive(Gamepad driverGamepad);
    void teleOpArcadeDrive(Gamepad driverGamepad, F310JoystickInputNames.Joysticks selectedDrivingStick);
    //We would add more mandatory functions for drivetrains
    //void getCurrentAverageDTPosition(boolean countStoppedSides); //TODO account for turning, straight lines after turning. Maybe, for specific ,straight segments, zero out Position before counting?
    int getCurrentAverageDTPosition(boolean countStoppedSides); //TODO account for turning, straight lines after turning. Maybe, for specific ,straight segments, zero out Position before counting?
    //private void setModeOfAllMotors(final DcMotor.RunMode runMode); Can't be done until Java 9 is used :(
}
