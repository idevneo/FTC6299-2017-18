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

@Autonomous(name = "AutoRedRight", group = "Sensor")
                            // Comment this out to add to the opmode list
public class AutoRedRight extends MyOpMode {
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        hMap(hardwareMap);
        // Set up our telemetry dashboard
//        composeTelemetry();
        // Wait until we're told to go

        waitForStart();
        runtime.reset();
/**---------------------------------------------------------------------------------------------------------------*/
        // Start the logging of measured acceleration
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

        vfValue();

        jewelArm.setPosition(.6);
        jewelHand.setPosition(.4);
        sleep(750);
        jewelArm.setPosition(.15);
        sleep(1000);
        if (jewelColor.red() > jewelColor.blue()) {
            jewelHand.setPosition((.3));
        } else if (jewelColor.red() < jewelColor.blue()) {
            jewelHand.setPosition((.6));
        }
        sleep(500);

        jewelArm.setPosition(.6);
        jewelHand.setPosition(.45);
        sleep(1000);
        jewelHand.setPosition(.3);
        sleep(500);

        setMotors(.25,.25);
        sleep(1000);
        stopMotors();

        rangeMoveStrafe(3, rangeR, 0);
        sleep(1000);
        rangeMovePID(8, rangeF);
        sleep(500);
//        try {
//            turn(.25, 0.1);
//        } catch (InterruptedException e) {
//            e.printStackTrace();
//        }
//        sleep(1000);
        rangeMoveStrafe(26.75, rangeR, 0);
        sleep(750);

        vfMovePerp('r', rangeR,0);

        manipAuto(-.75);
        sleep(500);

        manipAuto(-.75);

//back up, push forward, back up
        setMotors(-.2, -.2);
        sleep(250);
        stopMotors();

        manip.setPower(-1);
        setMotors(.4, .4);
        sleep(250);
        stopMotors();

        setMotors(-.2, -.2);
        sleep(250);
        stopMotors();
        manip.setPower(0);
    }
}
