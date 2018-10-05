package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class fourWheelDriveTrain {

    // REVTrix specific measurements
    static final double     COUNTS_PER_MOTOR_REV    = 0 ;
    static final double     DRIVE_GEAR_REDUCTION    = 0.0 ;

    // REVTrix specific drive train members.
    static final double     WHEEL_DIAMETER_INCHES   = 0.0 ; //estimate
    static final double     DRIVE_WHEEL_SEPARATION  = 0.0 ; //estimate
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    // REVTrix specific motor and actuator members.
    public DcMotor FrontLeftDrive   = null;
    public DcMotor FrontRightDrive  = null;
    public DcMotor RearLeftDrive    = null;
    public DcMotor RearRightDrive   = null;

    public void fourWheelDriveTrain( double counts_per_motor_rev, double drive_gear_reduction, double wheel_diamter, double drive_wheel_seperation){




    }

    public void Drive(){

    }

    public void initialize(HardwareMap ahwmap){

        FrontLeftDrive = ahwmap.get(DcMotor.class, "motor0");
        FrontRightDrive = ahwmap.get(DcMotor.class, "motor1");
        RearLeftDrive = ahwmap.get(DcMotor.class, "motor2");
        RearRightDrive = ahwmap.get(DcMotor.class, "motor3");

        FrontLeftDrive.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if
        // using AndyMark motors
        FrontRightDrive.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if
        // using AndyMark motors
        RearLeftDrive.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if
        // using AndyMark motors
        RearRightDrive.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if
        // using AndyMark motors

        // Set all motors to zero power
        FrontLeftDrive.setPower(0);
        FrontRightDrive.setPower(0);
        RearLeftDrive.setPower(0);
        RearRightDrive.setPower(0);

        // Set both motors to run with encoders.
        FrontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FrontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RearLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RearRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}

