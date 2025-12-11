package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;


@Autonomous
public class AutoBlueNear extends LinearOpMode {
    // Constants
    double TICKS_PER_ROTATION = 537.6;
    double WHEEL_DIAMETER = 96 / 25.4; // mm to inches
    double TICKS_PER_INCH = TICKS_PER_ROTATION / (Math.PI * WHEEL_DIAMETER);

    @Override
    public void runOpMode() {
        DcMotor leftFront = hardwareMap.get(DcMotor.class, "LeftFrontMotor");
        DcMotor rightFront = hardwareMap.get(DcMotor.class, "RightFrontMotor");
        DcMotor leftBack = hardwareMap.get(DcMotor.class, "LeftBackMotor");
        DcMotor rightBack = hardwareMap.get(DcMotor.class, "RightBackMotor");
        DcMotor outtakeWheel = hardwareMap.get(DcMotor.class, "OuttakeWheel");
        DcMotor outtakeWheel2 = hardwareMap.get(DcMotor.class, "OuttakeWheel2");
        BNO055IMU imu = hardwareMap.get(BNO055IMU.class, "imu");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.mode = BNO055IMU.SensorMode.IMU;

        imu.initialize(parameters);

        AutoUtils robot = new AutoUtils(leftFront, rightFront, leftBack, rightBack, imu, outtakeWheel, outtakeWheel2);
        waitForStart();

        while (opModeIsActive()) {
            robot.autoDrive(-1, -67);
            robot.autoShoot(1, 10000);
        }
    }

}
