package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class FourWheelDriveTrain extends ModularDriveTrain{
    // REVTrix specific motor and actuator members.
    private DcMotor FrontLeftDrive   = null;
    private DcMotor FrontRightDrive  = null;
    private DcMotor RearLeftDrive    = null;
    private DcMotor RearRightDrive   = null;

    //current motor power


    FourWheelDriveTrain(double COUNTS_PER_MOTOR_REV, double DRIVE_GEAR_REDUCTION, double WHEEL_DIAMETER_INCHES, double DRIVE_WHEEL_SEPARATION, DcMotor.RunMode RUNMODE){
        super(COUNTS_PER_MOTOR_REV, DRIVE_GEAR_REDUCTION, WHEEL_DIAMETER_INCHES, DRIVE_WHEEL_SEPARATION, RUNMODE);
    }

    /*public void DriveForwardInches(){

    }
    */

    public void init(HardwareMap ahwMap){

        FrontLeftDrive = ahwMap.get(DcMotor.class, "motor0");
        FrontRightDrive = ahwMap.get(DcMotor.class, "motor1");
        RearLeftDrive = ahwMap.get(DcMotor.class, "motor2");
        RearRightDrive = ahwMap.get(DcMotor.class, "motor3");

        FrontLeftDrive.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if
        // using AndyMark motors
        FrontRightDrive.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if
        // using AndyMark motors
        RearLeftDrive.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if
        // using AndyMark motors
        RearRightDrive.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if
        // using AndyMark motors

        // Set all motors to zero power
        FrontLeftDrive.setPower(0);
        FrontRightDrive.setPower(0);
        RearLeftDrive.setPower(0);
        RearRightDrive.setPower(0);

        if(RUNMODE == DcMotor.RunMode.RUN_USING_ENCODER){  //stops motors until mode changed; so do before setting runmode to RUN_USING_Encoder
            FrontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            FrontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            RearLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            RearRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }

        // Set both motors to run with encoders.
        FrontLeftDrive.setMode(RUNMODE);
        FrontRightDrive.setMode(RUNMODE);
        RearLeftDrive.setMode(RUNMODE);
        RearRightDrive.setMode(RUNMODE);


    }

    public void teleOpTankDrive(Gamepad driverGamepad){
        // Run wheels in tank mode (note: The joystick goes negative when pushed forwards,
        // so negate it) Negate it at function call
        leftPower = -driverGamepad.left_stick_y;
        rightPower = -driverGamepad.right_stick_y;
        FrontLeftDrive.setPower(leftPower);
        FrontRightDrive.setPower(rightPower);
        RearLeftDrive.setPower(leftPower);
        RearRightDrive.setPower(rightPower);
    }

    public void teleOpArcadeDrive(Gamepad driverGamepad, F310JoystickInputNames.Joysticks selectedDrivingStick){

        //float leftPower = 0f; //may want to move these as field variables. Probably should not initialize every time.
        //float rightPower = 0f;

        if(selectedDrivingStick == F310JoystickInputNames.Joysticks.LEFT_STICK){
            leftPower = -driverGamepad.left_stick_y - -driverGamepad.left_stick_x;
            rightPower = -driverGamepad.left_stick_y + -driverGamepad.left_stick_x;
        }

        else if(selectedDrivingStick == F310JoystickInputNames.Joysticks.RIGHT_STICK) {
            leftPower = -driverGamepad.right_stick_y - -driverGamepad.right_stick_x;
            rightPower = -driverGamepad.right_stick_y + -driverGamepad.right_stick_x;
        }

        FrontLeftDrive.setPower(leftPower);
        FrontRightDrive.setPower(rightPower);
        RearLeftDrive.setPower(leftPower);
        RearRightDrive.setPower(rightPower);
    }

    public int getAverageDTRotation(boolean countStoppedDTSide) { //TODO Work on getting encoder readins
        if (countStoppedDTSide) {
            return (FrontLeftDrive.getCurrentPosition() + FrontRightDrive.getCurrentPosition() +
                    RearLeftDrive.getCurrentPosition() + RearRightDrive.getCurrentPosition()) / 4;
        } else {
            return -1;

        }
    }
}

