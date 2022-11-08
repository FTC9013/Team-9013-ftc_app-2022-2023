package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Primary Tele-Op", group="Linear Opmode")
//@Disabled
public class TeleOpPrimary extends LinearOpMode
{
  // Declare OpMode members.
  private MecanumDriveChassis driveChassis;
  private GripperControl gripperControl;
  private ArmControl armcontrol;
 // private ManipulatorPlatform manipulatorPlatform;
  //private LEDs leds;
  
  enum keyOwner { RIGHT, LEFT, NONE };
  keyOwner owner = keyOwner.NONE;
  
  /*
  private final double shooterSpeedFull = 130;
  private final double shooterSpeedTolerance = 20;
  private final double shooterSpeedStop = 0;
  */
  
  private ElapsedTime runtime = new ElapsedTime();
  // a timer for the various automation activities.
  private ElapsedTime eventTimer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);

  
  //private final double colorSpeed = 10; // change colors every 10 seconds
  //private double LEDTimeoutTime = 0;
  
  
  @Override
  public void runOpMode()
  {
    
    
    telemetry.addData("Status", "Initialized");
    telemetry.update();
    
    // setup a instance of our drive system
    driveChassis = new MecanumDriveChassis(hardwareMap, telemetry);
    gripperControl = new GripperControl(hardwareMap, telemetry);
    armcontrol = new ArmControl(hardwareMap, telemetry);
    
    
    // set dead zone to minimize unwanted stick input.
    //gamepad1.setJoystickDeadzone((float)0.05);
    boolean goingFast = false;
    boolean goingFastToggle = false;
    
    
    // Wait for the game to start (driver presses PLAY)
    waitForStart();
    runtime.reset();
    //leds.goConfetti();  // start the LEDs in confetti
  
   // driveChassis.setSpeedScale(manipulatorPlatform.lowSpeed);
  
    // run until the end of the match (driver presses STOP)
    while (opModeIsActive())
    {
      telemetry.addData("LStickY", gamepad1.left_stick_y * -1);
      telemetry.addData("LStickX", gamepad1.left_stick_x);
      telemetry.update();
      //set up counter
      // steps through color sequences every 'colorSpeed' seconds.
      /*
      if ( eventTimeOut( LEDTimeoutTime ) )
      {
        LEDTimeoutTime = eventTimer.time() + colorSpeed;  // load for next cycle
        //leds.goChangeColor();
      }
      */
      // send joystick inputs to the drive chassis
      driveChassis.drive(gamepad1.left_stick_y, gamepad1.left_stick_x,
        gamepad1.right_stick_x, gamepad1.right_bumper);
  
      if (gamepad2.a)
      {
        gripperControl.pushOut();
      } else if (gamepad2.x)
      {
        gripperControl.pullIn();
      } else
      {
        gripperControl.stop();
      }
  
      if (gamepad2.dpad_up)
      {
        armcontrol.raise();
      }
      if (gamepad2.dpad_down)
      {
        armcontrol.lower();
      }
    }
      
      
     /* // *** Driver controls (game pad 1)
      // provide a throttle capability to run the bot at one of two speeds.
        if (gamepad1.right_bumper && !goingFast && !goingFastToggle)  // Go fast
        {
          driveChassis.setSpeedScale(1.0);
         // driveChassis.setSpeedScale(manipulatorPlatform.highSpeed);
          goingFast = true;
          goingFastToggle = true;
        }
        if (gamepad1.right_bumper && goingFast && !goingFastToggle)
        {
         // driveChassis.setSpeedScale(manipulatorPlatform.lowSpeed);
          driveChassis.setSpeedScale(0.5);
          goingFast = false;
          goingFastToggle = true;
        }
        
        if  (!gamepad1.right_bumper)
        {
          goingFastToggle = false;
        }
    
        */
        
        
        
       /* if(gamepad2.right_bumper && owner == keyOwner.NONE)
        {
          manipulatorPlatform.setSpinnerPower(1);
          owner = keyOwner.RIGHT;
          //while(gamepad2.right_bumper  )
          //{BVX
          //}
          //manipulatorPlatform.setSpinnerPower(0);
        }
        else if(!gamepad2.right_bumper && owner == keyOwner.RIGHT)
        {
          manipulatorPlatform.setSpinnerPower(0);
          owner = keyOwner.NONE;
        }*/
        
        
        
        
        /*if(gamepad2.left_bumper && owner == keyOwner.NONE)
        {
          manipulatorPlatform.setSpinnerPower(-1);
          owner = keyOwner.LEFT;
          //while(gamepad2.left_bumper)
          //{
          //}
          //manipulatorPlatform.setSpinnerPower(0);
        }
        else if(!gamepad2.left_bumper && owner == keyOwner.LEFT)
        {
          manipulatorPlatform.setSpinnerPower(0);
          owner = keyOwner.NONE;
        }*/
  
  

        
        
/*
        eventTimer.reset();
        while (opModeIsActive() && eventTimer.time() < 0.5)
        {
          driveChassis.autoDrive(telemetry);
        }*/
        //manipulatorPlatform.shooterExtend(shooterRetract);

      
      
      // ***************************
      // Second seat...  controls (game pad 2)
/*
// D‐PAD – Controls
      if (gamepad2.dpad_down)
      {
        manipulatorPlatform.setArmPosition(manipulatorPlatform.armGather);
      }
      if (gamepad2.dpad_right)
      {
        manipulatorPlatform.setArmPosition(manipulatorPlatform.armLow);
      }
      if (gamepad2.dpad_left)
      {
        manipulatorPlatform.setArmPosition(manipulatorPlatform.armMid);
      }
      if (gamepad2.dpad_up)
      {
        manipulatorPlatform.setArmPosition(manipulatorPlatform.armMax);
      }
      
      
      if(gamepad2.a)
      {
        manipulatorPlatform.setGatherPower(0.33);
      }
      if(gamepad2.y)
      {
        manipulatorPlatform.setGatherPower(-0.33);
      }
      if(gamepad2.b)
      {
        manipulatorPlatform.setGatherPower(0);
      }
      if(gamepad2.x)
      {
        manipulatorPlatform.invertGatherDropper();
      }*/
/*      // provide a
      if (gamepad1.right_trigger > 0.5 && gamepad1.left_trigger > 0.5  )  //
      {
        // Do something when both triggers are pressed.
      }
      else if (gamepad1.right_trigger < 0.5 || gamepad1.left_trigger < 0.5 )
      {
        // Do the other thing you do when both triggers are not held down.
      }*/

    }
  
  
  // test the event time
  private boolean eventTimeOut ( double eventTime)
  {
    return eventTimer.time() > eventTime;
  }
  
  
}