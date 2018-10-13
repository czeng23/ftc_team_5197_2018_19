package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

public class modularFTCBot extends GenericFTCRobot { //implement Modularizablehardware if you want. It is unecesarry however as each module implements
    // REVTrix specific measurements(maybe create final class to organize these constants)
    static final double     COUNTS_PER_MOTOR_REV    = 4 ;
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;

    // REVTrix specific drive train members.
    static final double     WHEEL_DIAMETER_INCHES   = 3.5 ; //estimate
    static final double     DRIVE_WHEEL_SEPARATION  = 28.0 ; //estimate
    static final DcMotor.RunMode RUNMODE = DcMotor.RunMode.RUN_USING_ENCODER;

    FourWheelDriveTrain dt = new FourWheelDriveTrain(COUNTS_PER_MOTOR_REV, DRIVE_GEAR_REDUCTION,
            WHEEL_DIAMETER_INCHES, DRIVE_WHEEL_SEPARATION, RUNMODE);


}
