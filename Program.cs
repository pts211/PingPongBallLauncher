/**
 * Use the mini-USB cable to deploy/debug.
 *  
 *  Right click on "PingPongBallLauncher" go to properties.
 *  Go to ".NET Micro Framework" and make sure the "Device" dropdown has
 *  the HERO board selected. 
 *
 *  Make sure PDB power is on before running.
 *  
 *  To Deploy/Run -> Click the "Start" button
 *  Click the "Stop" button to sotp debugging.
 * 
 */

using System;
using Microsoft.SPOT;
using CTRE;
using System.Threading;
using System.Text;
using CTRE.Phoenix.MotorControllers;
using CTRE.Phoenix.FRC;
using CTRE.Phoenix;


namespace PingPongBallLauncher
{
    public class Program
    {
        static RobotApplication _robotApp = new RobotApplication();

        public static void Main()
        {
            _robotApp.init();

            while (true)
            {
                _robotApp.run();

                Thread.Sleep(10);
            }
        }
    }

    public struct CupPosition
    {
        public int angle, speed;

        public CupPosition(int ang, int spd)
        {
            angle = ang;
            speed = spd;
        }
    }

    public class RobotApplication
    {
        /* ************************************************************** */
        /* ******************** VALUES TO MODIFY ******************** */
        /* ************************************************************** */

        //Ball Feed Delay
        const int FEED_DELAY = 250;

        //Shooter PID Values
        const float SHOOTER_P = 0.05f;
        const float SHOOTER_I = 0.0f;
        const float SHOOTER_D = 2.5f;
        const float SHOOTER_F = 0.0322f;

        //Turret PID Values
        const float TURRET_P = 0.1f;
        const float TURRET_I = 0.0f;
        const float TURRET_D = 0.01f;
        const float TURRET_F = 0.0f;

        const float MINIMUM_TURRET_VOLTAGE = 1.0f; //volts

        //Values used in MANUAL mode. The amount that the RPM or angle
        //will increase/decrease.
        const int RPM_INCREMENT = 100;
        const int ANGLE_INCREMENT = 1;

        const int STARTING_RPM = 0;     //The RPM that the shooter will start at in MANUAL mode.

        //void initCupSettings() <-- Modify values in this function (below) to change all the setpoints.

        /* ************************************************************** */
        /* ******************** END VALUES TO MODIFY ******************** */
        /* ************************************************************** */

        int activeCup = 0;
        CupPosition[] cupPositions = new CupPosition[10];

        enum MODE { MANUAL, PRESET };

        MODE currentMode = MODE.MANUAL;

        const int NUM_BUTTONS = 12;
        const int NUM_AXIS = 2;
        enum JOYSTICK : uint { X = 1, A = 2, B = 3, Y = 4, LB = 5, RB = 6, LT = 7, RT = 8, BACK = 9, START = 10, LSTICK_BTN = 11, RSTICK_BTN = 12, XAXIS_L = 13, XAXIS_R = 14, YAXIS_U = 15, YAXIS_D = 16 };
        const double STICK_TOLERANCE = 0.2;

        const double POSITIONAL_RATIO = ((1.0 / 360.0) * (48.0 / 28.0));
        const double SHOOTER_RATIO = 1.0f;

        const double MAX_SPEED = 5000;
        const int MAX_ANGLE = 90;

        bool isRunning = false;
        bool isPrintEnabled = false;

        TalonSrx _shooter = new TalonSrx(1);
        TalonSrx _turret = new TalonSrx(2);

        PneumaticControlModule _pcm = new PneumaticControlModule(3);

        /** Use a USB gamepad plugged into the HERO */
        CTRE.Phoenix.Controller.GameController _gamepad = new CTRE.Phoenix.Controller.GameController(CTRE.Phoenix.UsbHostDevice.GetInstance(0), 0);

        /** hold the current button values from gamepad*/
        bool[] _btns = new bool[16 + 1];
        /** hold the last button values from gamepad, this makes detecting on-press events trivial */
        bool[] _btnsLast = new bool[16 + 1];

        int rpmSetpoint = STARTING_RPM;
        int angleSetpoint = 0;

        float scaledValue = 0;

        public void init()
        {
            initCupSettings();
            initTurret();
            initShooter();

            Thread.Sleep(100);  //wait to make sure the positions take effect
        }

        void initCupSettings()
        {

            // SPEEDS AND ANGLES
            /*
              9   8   7   6  
                5   4   3  
                  2   1  
                    0 
            */
            cupPositions[0].speed = 2000; //RPM
            cupPositions[0].angle = -5;    //DEGREES

            cupPositions[1].speed = 2000; //RPM
            cupPositions[1].angle = -15;    //DEGREES

            cupPositions[2].speed = 2000; //RPM
            cupPositions[2].angle = -10;    //DEGREES

            cupPositions[3].speed = 2000; //RPM
            cupPositions[3].angle = 0;    //DEGREES

            cupPositions[4].speed = 2000; //RPM
            cupPositions[4].angle = -20;    //DEGREES

            cupPositions[5].speed = 2000; //RPM
            cupPositions[5].angle = -21;    //DEGREES

            cupPositions[6].speed = 2000; //RPM
            cupPositions[6].angle = -30;    //DEGREES

            cupPositions[7].speed = 2000; //RPM
            cupPositions[7].angle = -20;    //DEGREES

            cupPositions[8].speed = 2000; //RPM
            cupPositions[8].angle = -12;    //DEGREES

            cupPositions[9].speed = 2000; //RPM
            cupPositions[9].angle = 0;    //DEGREES
        }

        void initTurret()
        {
            /* first choose the sensor */
            _turret.SetFeedbackDevice(TalonSrx.FeedbackDevice.CtreMagEncoder_Relative);
            _turret.SetSensorDirection(false);
            _turret.ConfigEncoderCodesPerRev(4096); // if using CTRE.TalonSrx.FeedbackDevice.QuadEncoder

            _turret.SetP(0, TURRET_P); /* tweak this first, a little bit of overshoot is okay */
            _turret.SetI(0, TURRET_I);
            _turret.SetD(0, TURRET_D);
            _turret.SetF(0, TURRET_F);

            /* use slot0 for closed-looping */
            _turret.SelectProfileSlot(0);

            /* set the peak and nominal outputs, 12V means full */
            _turret.ConfigNominalOutputVoltage(MINIMUM_TURRET_VOLTAGE, -1 * MINIMUM_TURRET_VOLTAGE); //The minimum voltage that will be applied to the turret.
            _turret.ConfigPeakOutputVoltage(+3.0f, -3.0f);      //THe maximum voltage that will be applied to the turret.

            /* how much error is allowed?  This defaults to 0. */
            _turret.SetAllowableClosedLoopErr(0, 0);

            _turret.SetPosition(0); /* start our position at zero, this example uses relative positions */
            _turret.SetVoltageRampRate(0); /* V per sec */

            _turret.SetControlMode(ControlMode.kPosition);
            _turret.Set(angleSetpoint);

            _turret.SetEncPosition(0);
        }

        void initShooter()
        {
            /* first choose the sensor */
            _shooter.SetFeedbackDevice(TalonSrx.FeedbackDevice.CtreMagEncoder_Relative);
            _shooter.SetSensorDirection(false);

            _shooter.SetControlMode(ControlMode.kSpeed);
            _shooter.ConfigNominalOutputVoltage(+0.65f, -0.65f); //The minimum voltage that will be applied to the shooter.
            _shooter.ConfigPeakOutputVoltage(+12.0f, -12.0f);    //THe maximum voltage that will be applied to the shooter.
            _shooter.Set(0);

            _shooter.SetP(0, SHOOTER_P); /* tweak this first, a little bit of overshoot is okay */
            _shooter.SetI(0, SHOOTER_I);
            _shooter.SetD(0, SHOOTER_D);
            _shooter.SetF(0, SHOOTER_F);

            /* use slot0 for closed-looping */
            _shooter.SelectProfileSlot(0);

        }

        public void run()
        {
            Loop10Ms();

            //Enable or Disable the entire system using the START and BACK buttons.
            if (_gamepad.GetButton((int)JOYSTICK.START))
            {
                Debug.Print("ENABLING!!!!!");
                isRunning = true;
            }
            else if (_gamepad.GetButton((int)JOYSTICK.BACK))
            {
                Debug.Print("Disabled");
                isRunning = false;
            }
            if (isRunning)
            {
                CTRE.Phoenix.Watchdog.Feed();
            }
        }

        void Loop10Ms()
        {
            /* get all the buttons */
            FillBtns(ref _btns);

            //MAX RPM: ~4500

            if (CheckButton(JOYSTICK.X))
            {
                rpmSetpoint -= RPM_INCREMENT;
            }

            if (CheckButton(JOYSTICK.B))
            {
                rpmSetpoint += RPM_INCREMENT;
            }

            if (CheckButton(JOYSTICK.A))
            {
                angleSetpoint -= ANGLE_INCREMENT;
            }
            
            if (CheckButton(JOYSTICK.Y))
            {
                angleSetpoint += ANGLE_INCREMENT;
            }

            if (CheckButton(JOYSTICK.RB))
            {
                if(currentMode == MODE.PRESET && activeCup < 9)
                {
                    activeCup++;
                }
                PrintBeerPong();
            }

            if (CheckButton(JOYSTICK.LB))
            {
                if (currentMode == MODE.PRESET && activeCup > 0)
                {
                    activeCup--;
                }
                PrintBeerPong();
            }

            if (CheckButton(JOYSTICK.RT))
            {
                ToggleMode();
            }

            if (CheckButton(JOYSTICK.LT))
            {
                isPrintEnabled = !isPrintEnabled;
            }

            if (CheckButton(JOYSTICK.YAXIS_D))
            {
                FeedBall();
            }

            if (isPrintEnabled)
            {
                Print_All();
            }

            SetShooterAndTurret();

            //Print_Speed();
            //Print_Rotations();
            //Print_Buttons();

            /* copy btns => btnsLast */
            System.Array.Copy(_btns, _btnsLast, _btns.Length);
        }

        bool CheckButton(JOYSTICK btn)
        {
            //int btnChk = (int)btn;
            //Debug.Print("btn[" + btnChk + "];" + _btns[btnChk]);
            return _btns[(int)btn] && !_btnsLast[(int)btn];
        }

        void ToggleMode()
        {
            if(currentMode == MODE.MANUAL)
            {
                currentMode = MODE.PRESET;

                StringBuilder _ms = new StringBuilder();
                _ms.Clear();
                _ms.Append(" ****************************************\n");
                _ms.Append(" CURRENT MODE: PRESET\n");
                _ms.Append(" ****************************************\n");
                Debug.Print(_ms.ToString());
                PrintBeerPong();
            }
            else
            {
                currentMode = MODE.MANUAL;
                StringBuilder _ms = new StringBuilder();
                _ms.Clear();
                _ms.Append(" ****************************************\n");
                _ms.Append(" CURRENT MODE: MODE\n");
                _ms.Append(" ****************************************\n");
                Debug.Print(_ms.ToString());
            }
        }

        void SetShooterAndTurret()
        {
            if(currentMode == MODE.MANUAL)
            {
                SetShooterSpeed(rpmSetpoint);
                SetTurretAngle(angleSetpoint);
            }
            else if(currentMode == MODE.PRESET)
            {
                SetShooterSpeed(cupPositions[activeCup].speed);
                SetTurretAngle(cupPositions[activeCup].angle);
            }
        }

        void SetShooterSpeed(int speed)
        {
            if (speed > 0 && speed < MAX_SPEED)
            {
                _shooter.Set(speed);
            }
        }

        void SetTurretAngle(int angle)
        {
            if(angle > MAX_ANGLE)
            {
                angle = MAX_ANGLE;
            }
            else if(angle < (MAX_ANGLE * -1) )
            {
                angle = MAX_ANGLE * -1;
            }
            _turret.Set(degreesToScaledUnit(angle));
        }

        void FeedBall()
        {
            _pcm.SetSolenoidOutput(0, true);
            Thread.Sleep(FEED_DELAY);  //Amount of time to keep the ball feed open.
            _pcm.SetSolenoidOutput(0, false);
        }
       
        /** throw all the gamepad buttons into an array */
        void FillBtns(ref bool[] btns)
        {
            for (uint i = 1; i <= NUM_BUTTONS; ++i)
            {
                btns[i] = _gamepad.GetButton(i);
            }

            if (_gamepad.GetAxis(0) < -1.0 + STICK_TOLERANCE)
            {
                btns[(int)JOYSTICK.XAXIS_L] = true;
                btns[(int)JOYSTICK.XAXIS_R] = false;
            }
            else if (_gamepad.GetAxis(0) > 1.0 - STICK_TOLERANCE)
            {
                btns[(int)JOYSTICK.XAXIS_L] = false;
                btns[(int)JOYSTICK.XAXIS_R] = true;
            }
            else
            {
                btns[(int)JOYSTICK.XAXIS_L] = false;
                btns[(int)JOYSTICK.XAXIS_R] = false;
            }

            if (_gamepad.GetAxis(1) < -1.0 + STICK_TOLERANCE)
            {
                btns[(int)JOYSTICK.YAXIS_U] = true;
                btns[(int)JOYSTICK.YAXIS_D] = false;
            }
            else if (_gamepad.GetAxis(1) > 1.0 - STICK_TOLERANCE)
            {
                btns[(int)JOYSTICK.YAXIS_U] = false;
                btns[(int)JOYSTICK.YAXIS_D] = true;
            }
            else
            {
                btns[(int)JOYSTICK.YAXIS_U] = false;
                btns[(int)JOYSTICK.YAXIS_D] = false;
            }
        }

        /** CONVERSIONS **/
        float degreesToScaledUnit(float deg)
        {
            //Final Ratio: 28:48
            return (float)(deg * POSITIONAL_RATIO);
        }

        double scaledUnitsToDegrees(double ticks)
        {
            return (ticks / POSITIONAL_RATIO);
        }

        /** DEBUG LOGGING FUNCTIONS **/

        void PrintBeerPong()
        {
            StringBuilder _ms = new StringBuilder();
            _ms.Clear();
            _ms.Append("\n\n");

            int noOfRows = 4;
            int rowCount = 4;
            int counter = 5;
            int index = 0;

            for (int i = 0; i < noOfRows; i++)
            {
                //Printing i spaces at the beginning of each row
                for (int j = 1; j <= i; j++)
                {
                    _ms.Append("  ");
                }
                for (int j = 1; j <= rowCount; j++)
                {
                    index = counter + rowCount;
                    if (index == activeCup)
                    {
                        _ms.Append("[" + index + "] ");
                    }
                    else
                    {
                        _ms.Append(" " + index + "  ");
                    }

                    if (j != rowCount)
                    {
                        counter--;
                    }
                }
                _ms.Append("\n");
                rowCount--;
            }
            _ms.Append("\n");

            _ms.Append(" Target Turret Angle: " + cupPositions[activeCup].angle + " degrees | Target Shooter Speed: " + cupPositions[activeCup].speed + " RPM ");

            Debug.Print(_ms.ToString());
        }
        
        void Print_All()
        {
            StringBuilder _ms = new StringBuilder();
            _ms.Clear();
            _ms.Append(" Target Angle=");
            _ms.Append(angleSetpoint);
            _ms.Append("  Current Angle=");
            _ms.Append(scaledUnitsToDegrees(_turret.GetEncPosition()));
            _ms.Append(" Target RPM=");
            _ms.Append(rpmSetpoint);
            _ms.Append(" Current RPM=");
            _ms.Append(_shooter.GetSpeed());
            Debug.Print(_ms.ToString());
        }

        void Print_Rotations()
        {
            StringBuilder _ms = new StringBuilder();
            _ms.Clear();
            _ms.Append(" angleSetpoint=");
            _ms.Append(angleSetpoint);
            _ms.Append("_t.GetEncPosition()=");
            _ms.Append(_turret.GetEncPosition());
            _ms.Append("  GetPositionInDegrees()=");
            _ms.Append(scaledUnitsToDegrees(_turret.GetEncPosition()));

            Debug.Print(_ms.ToString());
        }

        void Print_Speed()
        {
            StringBuilder _ms = new StringBuilder();
            _ms.Clear();
            _ms.Append(" rpmSetpoint=");
            _ms.Append(rpmSetpoint);
            _ms.Append("_s.GetSpeed()=");
            _ms.Append(_shooter.GetSpeed());
            _ms.Append(" _s.GetEncPosition()=");
            _ms.Append(_shooter.GetEncPosition());
            _ms.Append(" _s.GetEncVel()=");
            _ms.Append(_shooter.GetEncVel());

            Debug.Print(_ms.ToString());
        }

        void Print_Buttons()
        {
            StringBuilder _btnStr = new StringBuilder();

            _btnStr.Clear();
            for (uint i = 1; i < _btns.Length; ++i)
            {
                _btnStr.Append("btn[");
                _btnStr.Append(((JOYSTICK)i).ToString());
                _btnStr.Append("]: ");
                _btnStr.Append(_btns[i]);
                _btnStr.Append(" ");
            }

            Debug.Print(_btnStr.ToString());
        }
    }
}
