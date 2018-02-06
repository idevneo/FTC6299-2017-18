package org.firstinspires.ftc.teamcode.Test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.teamcode.Library.MyOpMode;

@Autonomous(name = "Speed Test aut", group = "Sensor") // Comment this out to add to the opmode list
public class SpeedTest extends MyOpMode {
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        hMap(hardwareMap);
        // Set up our telemetry dashboard
        composeTelemetry();
        // Wait until we're told to go

        waitForStart();
        runtime.reset();
/**---------------------------------------------------------------------------------------------------------------*/
        // Start the logging of measured acceleration
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

        vfValue();

        jewelArm.setPosition(.55);
        jewelHand.setPosition(.4);
        sleep(500);
        jewelArm.setPosition(.15);
        sleep(850);
        if (jewelColor.red() < jewelColor.blue()) {
            jewelHand.setPosition((.3));
        } else if (jewelColor.red() > jewelColor.blue()) {
            jewelHand.setPosition((.6));
        }
        sleep(750);

        jewelArm.setPosition(.55);
        jewelHand.setPosition(.45);
        sleep(300);
        jewelHand.setPosition(.3);
        sleep(300);
//.6s off

        setMotors(-.4, -.4);
        sleep(1000);
        stopMotors();
        sleep(100);

        try {
            turnCorr2(.5,-85, 7000); //pow upped
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        sleep(500);
//.5s

        setMotorStrafe(-.5); //.5
        sleep(800);
        stopMotors();

        setMotorStrafe(.4);
        sleep(825);
        stopMotors();

        vfMoveAlt();

        rangeMovePID(6, rangeF);

        manipAuto(-.75);
        sleep(500);

//back up, push forward, back up
        setMotors(-.2, -.2);
        sleep(250);
        stopMotors();

        manip.setPower(-1);
        setMotors(.3, .3);
        sleep(250);
        stopMotors();

        setMotors(-.2, -.2);
        sleep(250);
        stopMotors();
        manip.setPower(0);
        // Turn for second block

        try {
            turnCorr2(0.3,85, 7000); //changed power
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        sleep(500);
//.5s
        manip.setPower(1);
        rangeMovePID(4, rangeF);
        sleep(100);

        setMotors(.5,.5);
        sleep(200);
        stopMotors();

        //fishing - tc2 = .25
        try {
            turnCorr2(0.2,105, 1000);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        sleep(250);

        setMotors(.5,.5);
        sleep(200);
        stopMotors();

        try {
            turnCorr2(0.2,65, 1000);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        sleep(250);

        setMotors(.5,.5);
        sleep(200);
        stopMotors();

        try {
            turnCorr2(0.2,85, 1000);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        sleep(250);


//reeling
        manip.setPower(0);

        try {
            turnCorr2(0.1,-85, 7000);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        sleep(500);
//.5s
        rangeMovePID(6, rangeF);
        manipAuto(-.75);

        setMotors(-.2, -.2); //back up, push forward, back up
        sleep(250);
        stopMotors();

        manip.setPower(-1);
        setMotors(.3, .3);
        sleep(250);
        stopMotors();

        setMotors(-.2, -.2);
        sleep(250);
        stopMotors();
        manip.setPower(0);
    }
}