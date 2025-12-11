package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class AutoUtils {
    public DcMotor leftFront, rightFront, leftBack, rightBack, outtakeWheel, outtakeWheel2;
    public BNO055IMU imu;

    public static final double TICKS_PER_ROTATION = 537.6;
    public static final double WHEEL_DIAMETER = 96 / 25.4; // mm to inches
    public static final double TICKS_PER_INCH = TICKS_PER_ROTATION / (Math.PI * WHEEL_DIAMETER);
    private LinearOpMode opMode;

    public AutoUtils(DcMotor lf, DcMotor rf, DcMotor lb, DcMotor rb, BNO055IMU imu, DcMotor ow, DcMotor ow2) {
        leftFront = lf;
        rightFront = rf;
        leftBack = lb;
        rightBack = rb;
        outtakeWheel = ow;
        outtakeWheel2 = ow2;
        this.imu = imu;
    }

    public void autoDrive(double power, double inches) {
        int position = (int) (inches * TICKS_PER_INCH);

        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFront.setTargetPosition(position);
        rightFront.setTargetPosition(position);
        leftBack.setTargetPosition(position);
        rightBack.setTargetPosition(position);

        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftFront.setPower(power);
        rightFront.setPower(power);
        leftBack.setPower(power);
        rightBack.setPower(power);

        while (leftFront.isBusy() && rightFront.isBusy()) {}

        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);

        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void autoShoot(double power, double ticks) {
        outtakeWheel.setPower(1);
        opMode.sleep(1000);
        outtakeWheel2.setPower(1);

        outtakeWheel.setTargetPosition((int) ticks);
        outtakeWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        outtakeWheel.setPower(power);

        while (outtakeWheel.isBusy()) {}

        outtakeWheel.setPower(0);
        outtakeWheel2.setPower(0);
    }

    public void rotateToAngle(double angle) {

        // Normalize (-180 to 180)
        angle = ((angle + 180) % 360 + 360) % 360 - 180;

        final double kP = 0.012;
        final double maxPower = 0.5;
        final double tolerance = 1.0;

        while (true) {

            Orientation angles = imu.getAngularOrientation(
                    AxesReference.INTRINSIC,
                    AxesOrder.ZYX,
                    AngleUnit.DEGREES
            );

            double heading = angles.firstAngle;

            double error = angle - heading;
            error = ((error + 180) % 360 + 360) % 360 - 180;

            if (Math.abs(error) <= tolerance) {
                leftFront.setPower(0);
                leftBack.setPower(0);
                rightFront.setPower(0);
                rightBack.setPower(0);
                break;
            }

            double power = kP * error;
            power = Math.max(-maxPower, Math.min(maxPower, power));

            leftFront.setPower(power);
            leftBack.setPower(power);
            rightFront.setPower(power);
            rightBack.setPower(power);

            opMode.sleep(10);
        }
    }

}
