/**
 * Use the mini-USB cable to deploy/debug.
 *
 * 
 */

using System;
using Microsoft.SPOT;
using CTRE;
using System.Threading;
using System.Text;
using CTRE.Phoenix.MotorControllers;


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

    public class RobotApplication
    {
        //PID Values
        const float SHOOTER_P = 0.05f;
        const float SHOOTER_I = 0.0f;
        const float SHOOTER_D = 2.5f;
        const float SHOOTER_F = 0.0322f;

        const float TURRET_P = 0.02f;
        const float TURRET_I = 0.0f;
        const float TURRET_D = 0.01f;
        const float TURRET_F = 0.0f;

        const int NUM_BUTTONS = 12;
        const int NUM_AXIS = 2;
        enum JOYSTICK : uint { X = 1, A = 2, B = 3, Y = 4, LB = 5, RB = 6, LT = 7, RT = 8, BACK = 9, START = 10, LSTICK_BTN = 11, RSTICK_BTN = 12, XAXIS_L = 13, XAXIS_R = 14, YAXIS_U = 15, YAXIS_D = 16 };
        const double STICK_TOLERANCE = 0.2;

        const double POSITIONAL_RATIO = ((1.0 / 360.0) * (48.0 / 28.0));
        const double SHOOTER_RATIO = 1.0f;

        const double MAX_SPEED = 5000;

        bool isRunning = false;
        bool isClosedLoopEnabled = false;

        TalonSrx _turret = new TalonSrx(2);
        TalonSrx _shooter = new TalonSrx(1);

        /** Use a USB gamepad plugged into the HERO */
        CTRE.Phoenix.Controller.GameController _gamepad = new CTRE.Phoenix.Controller.GameController(CTRE.Phoenix.UsbHostDevice.GetInstance(0), 0);

        /** hold the current button values from gamepad*/
        bool[] _btns = new bool[16 + 1];
        /** hold the last button values from gamepad, this makes detecting on-press events trivial */
        bool[] _btnsLast = new bool[16 + 1];

        int rpmSetpoint = 0;
        int angleSetpoint = 0;

        float scaledValue = 0;

        public void init()
        {
            initTurret();
            initShooter();

            Thread.Sleep(100);  //wait to make sure the positions take effect
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
            _turret.ConfigNominalOutputVoltage(+0.65f, -0.65f); //The minimum voltage that will be applied to the turret.
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
            
            // X Button                   
            if (CheckButton(JOYSTICK.X))
            {
                //TEAM: Change this to modify how much the RPM changes each time the button is pressed.
                rpmSetpoint -= 100;
                SetShooterSpeed(rpmSetpoint);
            }

            if (CheckButton(JOYSTICK.B))
            {
                //TEAM: Change this to modify how much the RPM changes each time the button is pressed.
                rpmSetpoint += 100;
                SetShooterSpeed(rpmSetpoint);
            }

            if (CheckButton(JOYSTICK.A))
            {
                //TEAM: Change this to modify how many degrees the angle changes each time the button is pressed.
                angleSetpoint -= 1;
                scaledValue = degreesToScaledUnit(angleSetpoint);
                Debug.Print("Angle Setpoint: " + angleSetpoint + " Scaled value: " + scaledValue);
                _turret.Set(scaledValue);
            }
            
            if (CheckButton(JOYSTICK.Y))
            {
                //TEAM: Change this to modify how many degrees the angle changes each time the button is pressed.
                angleSetpoint += 1;
                scaledValue = degreesToScaledUnit(angleSetpoint);
                Debug.Print("Angle Setpoint: " + angleSetpoint + " Scaled value: " + scaledValue);
                _turret.Set(scaledValue);

            }

            if (CheckButton(JOYSTICK.RB))
            {
                //UNUSED   
            }

            if (CheckButton(JOYSTICK.LB))
            {
                //UNUSED
            }

            if (CheckButton(JOYSTICK.RT))
            {
                Print_NewRotation();
            }

            if (CheckButton(JOYSTICK.LT))
            {
                Print_SRX_Data();
            }

            if (isClosedLoopEnabled)
            {
                //GoToPositionInDegrees(angleSetpoint);
            }
            
            //Print_SRX_Data();
            //Print_Speed();
            Print_All();
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

        void SetShooterSpeed(float speed)
        {
            if (speed > 0 && speed < MAX_SPEED)
            {
                _shooter.Set(speed);
            }
        }
       
        /** throw all the gamepad buttons into an array */
        void FillBtns(ref bool[] btns)
        {
            for (uint i = 1; i <= NUM_BUTTONS; ++i)
            {
                //Debug.Print("i: " + i);
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

        void Print_SRX_Data()
        {
            StringBuilder _ms = new StringBuilder();
            _ms.Clear();
            _ms.Append(" CurrentPos=");
            _ms.Append(_turret.GetPosition());
            _ms.Append(" Output Voltage ");
            _ms.Append(_turret.GetOutputVoltage());


            Debug.Print(_ms.ToString());
        }

        void Print_All()
        {
            StringBuilder _ms = new StringBuilder();
            _ms.Clear();
            _ms.Append(" angleSetpoint=");
            _ms.Append(angleSetpoint);
            _ms.Append("_t.GetEncPosition()=");
            _ms.Append(_turret.GetEncPosition());
            _ms.Append("  GetPositionInDegrees()=");
            _ms.Append(scaledUnitsToDegrees(_turret.GetEncPosition()));
            _ms.Append(" rpmSetpoint=");
            _ms.Append(rpmSetpoint);
            _ms.Append(" _s.GetSpeed()=");
            _ms.Append(_shooter.GetSpeed());
            _ms.Append(" _s.GetEncPosition()=");
            _ms.Append(_shooter.GetEncPosition());
            _ms.Append(" _s.GetEncVel()=");
            _ms.Append(_shooter.GetEncVel());
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

        void Print_NewRotation()
        {
            StringBuilder _out = new StringBuilder();

            _out.Clear();
            _out.Append(" turretEncoder: " + _turret.GetEncPosition());
            _out.Append(" turretPosition: " + _turret.GetPosition());

            Debug.Print(_out.ToString());
        }
    }
}
