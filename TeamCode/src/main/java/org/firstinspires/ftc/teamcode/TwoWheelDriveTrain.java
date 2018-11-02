package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class TwoWheelDriveTrain extends ModularDriveTrain {
    private DcMotor leftDrive   = null;
    private DcMotor rightDrive  = null;

    TwoWheelDriveTrain(double COUNTS_PER_MOTOR_REV, double DRIVE_GEAR_REDUCTION, double WHEEL_DIAMETER_INCHES, double DRIVE_WHEEL_SEPARATION, DcMotor.RunMode RUNMODE){
        super(COUNTS_PER_MOTOR_REV, DRIVE_GEAR_REDUCTION, WHEEL_DIAMETER_INCHES, DRIVE_WHEEL_SEPARATION, RUNMODE);
    }

    public void init(HardwareMap ahwMap){
        leftDrive = ahwMap.get(DcMotor.class, "motor0");
        rightDrive = ahwMap.get(DcMotor.class, "motor1");

        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);

        leftDrive.setPower(0);
        rightDrive.setPower(0);

        leftDrive.setMode(RUNMODE);
        rightDrive.setMode(RUNMODE);
    }

    public void teleOpTankDrive(Gamepad driverGamepad) {
        // Run wheels in tank mode (note: The joystick goes negative when pushed forwards,
        // so negate it) Negate it at function call
        leftPower = -driverGamepad.left_stick_y; //may seem unnecessary to assign leftPower, but this is for consistency.
        rightPower = -driverGamepad.right_stick_y;

        leftDrive.setPower(leftPower);
        rightDrive.setPower(rightPower);
    }

    public void teleOpArcadeDrive(Gamepad driverGamepad, F310JoystickInputNames.Joysticks selectedDrivingStick) {

        if(selectedDrivingStick == F310JoystickInputNames.Joysticks.LEFT_STICK){ //in future, consider moving this to parent class
            leftPower = -driverGamepad.left_stick_y - -driverGamepad.left_stick_x;
            rightPower = -driverGamepad.left_stick_y + -driverGamepad.left_stick_x;
        }

        else if(selectedDrivingStick == F310JoystickInputNames.Joysticks.RIGHT_STICK) {
            leftPower = -driverGamepad.right_stick_y - -driverGamepad.right_stick_x;
            rightPower = -driverGamepad.right_stick_y + -driverGamepad.right_stick_x;
        }

        leftDrive.setPower(leftPower);
        rightDrive.setPower(rightPower);

    }

    @Override
    public int getCurrentAverageDTPosition(boolean countStoppedDTSides) { //TODO Work on getting encoder readins //

            return -1;
        }
}
