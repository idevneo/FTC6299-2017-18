package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.Library.MyOpMode;

@Autonomous(name = "AutoRedRight", group = "Linear Opmode")
public class AutoRedRight extends MyOpMode {
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        hMap(hardwareMap);
        while (!isStopRequested() && !imu.isGyroCalibrated()) {
            sleep(50);
            idle();
            telemetry.addLine("Initializing IMU...");
            telemetry.update();
        }
        align = true;
        // Set up our telemetry dashboard
//        composeTelemetry();

        // Wait until we're told to go
        waitForStart();
        runtime.reset();
/**---------------------------------------------------------------------------------------------------------------*/
        vfValue();
        jewelKnockerRed();

        setMotorsAll(.25,0,0);
        sleep(1000);
        stopMotors();

        setStrafeAuto(3.5, rangeR, 0 , 0);
//        rangeMoveStrafe(3.5, rangeR, 0);
        sleep(1000);
        rangeMovePID(10, rangeF);
        sleep(500);

        setStrafeAuto(26.75, rangeR, 0, 0);
//        rangeMoveStrafe(26.75, rangeR, 0);
        sleep(750);

        vfMovePar('r', rangeR,0);

        manipAuto(-.75);
        sleep(500);
        manipAuto(-.75);

//back up, push forward, back up
        setMotorsAll(-.2,0,0);
        sleep(250);
        stopMotors();

        manip.setPower(-1);
        setMotorsAll(.4,0,0);
        sleep(250);
        stopMotors();

        setMotorsAll(-.2,0,0);
        sleep(250);
        stopMotors();
        manip.setPower(0);
    }
}
