package org.firstinspires.ftc.teamcode.Test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.teamcode.Library.MyOpMode;

@Autonomous(name = "Speed Test aut", group = "Test") // Comment this out to add to the opmode list
public class SpeedTest extends MyOpMode {
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        hMap(hardwareMap);
        // Set up our telemetry dashboard
//        composeTelemetry();
        // Wait until we're told to go

        // Start the logging of measured acceleration
//        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000); //TEST COMMENTED OUT
        waitForStart();
        runtime.reset();
/**---------------------------------------------------------------------------------------------------------------*/
        vfValue();
        jewelKnockerBlue();

        setMotors(-.4, -.4);
        sleep(1000);
        stopMotors();
        sleep(100);

        try {
            turnCorr(.6,-85, 7000); //pow upped
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        sleep(500);

        setMotorStrafe(-.5); //.5
        sleep(800);
        stopMotors();

        setMotorStrafe(.4);
        sleep(825);
        stopMotors();

        vfMoveAlt();
        sleep(300);

        rangeMovePID(6.25, rangeF);

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

        setMotors(-.2, -.2);
        sleep(100);
        stopMotors();
        // Turn for second block

        try {
            turnCorr(0.4,87.5, 7000); //changed power
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        sleep(500);
//.5s
        manip.setPower(1);

        rangeMovePID(4.5, rangeF);
        setMotors(.5,.5);
        sleep( 350);
        stopMotors();

        //fishing - tc2 = .25
        try {
            turnCorr(0.4,105, 1000);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        sleep(250);

        setMotors(.2, .2);
        sleep(200);
        stopMotors();

        try {
            turnCorr(0.4,65, 1000);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        sleep(250);

        try {
            turnCorr(0.4,85, 1000);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        sleep(250);


//reeling
        manip.setPower(0);

        try {
            turnCorr(0.5,-85, 7000);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        sleep(500);
//.5s
        rangeMovePID(6, rangeF);
        
        liftLeft.setPower(-.5);
        liftRight.setPower(.5);
        sleep(500);
        liftLeft.setPower(0);
        liftRight.setPower(0);
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