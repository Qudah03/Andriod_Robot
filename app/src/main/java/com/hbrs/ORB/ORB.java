//*******************************************************************
/*!
\file   ORB.java
\author Thomas Breuer
\date   21.02.2019
\brief
*/

//*******************************************************************
package com.hbrs.ORB;

import android.app.Activity;
import android.bluetooth.BluetoothDevice;
import android.content.Context;
import android.content.Intent;
import android.hardware.usb.UsbManager;
import java.io.IOException;
import java.util.UUID;

//*******************************************************************
public class ORB extends ORB_RemoteHandler implements Runnable
{


    //---------------------------------------------------------------
    public final static byte POWER_MODE = 0;
    public final static byte BRAKE_MODE = 1;
    public final static byte SPEED_MODE  = 2;
    public final static byte MOVETO_MODE = 3;

    //---------------------------------------------------------------
    public final static byte NONE   = 0;
    public final static byte UART   = 1;
    public final static byte I2C    = 2;
    public final static byte ANALOG = 3;

    //---------------------------------------------------------------
    public final static int M1 = 0;
    public final static int M2 = 1;
    public final static int M3 = 2;
    public final static int M4 = 3;


    public ORB_RemoteUSB orb_USB;
    public ORB_RemoteBT  orb_BT;

    Thread mainThread;
    boolean runMainThread = false;

    //---------------------------------------------------------------
    public ORB( Activity parent )
    {
        orb_USB = new ORB_RemoteUSB(this);
        orb_BT = new ORB_RemoteBT(this);

        mainThread = new Thread(this);

        orb_USB.init((UsbManager) parent.getSystemService(Context.USB_SERVICE));
        orb_BT.init();

        if( mainThread.getState() == Thread.State.NEW )
        {
            runMainThread = true;
            mainThread.start();
            mainThread.setPriority(2);
        }
    }

    //-----------------------------------------------------------------
    public void openUSB(Intent intent)
    {
        orb_USB.open(intent);
    }

    //-----------------------------------------------------------------
    public boolean openBT(BluetoothDevice BT_Device )
    {
        if( BT_Device != null ) {
            orb_BT.open(BT_Device);
            try {
                Thread.sleep(500);
            } catch (InterruptedException e) {
                // Restore the interrupted status
                Thread.currentThread().interrupt();
            }
            // Return the actual connection status
            return orb_BT.isConnected();
        }
        return false;
    }

    //-----------------------------------------------------------------
    public void close()
    {
        orb_USB.close();
        orb_BT.close();

        runMainThread = false;
    }

    //-----------------------------------------------------------------
    public String getDeviceName()
    {
        if( orb_USB.isConnected() || orb_BT.isConnected())
            return(settingsFromORB.getName());
        else
            return( new String() );
    }

    //-----------------------------------------------------------------
    @Override
    public void run()
    {
        int timeout=0;

        while ( runMainThread )
        {
            if( orb_USB.isConnected() )
            {
                setORB_Remote(orb_USB);
            }
            else if (orb_BT.isConnected())
            {
                setORB_Remote(orb_BT);
            }
            else
            {
                // If neither is connected, do nothing or set a null remote if applicable.
                // For now, we can just skip the update logic.
                try {
                    Thread.sleep(100); // Sleep a bit longer when idle
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                }
                continue; // Skip the rest of the loop until a connection is made
            }

            if( update() )
            {
               // setMsgConnection(orb.settingsFromORB.getName());
                timeout = 0;
            }
            else
            {
                if( timeout < 1000)
                    timeout++;
                else {
                  //  setMsgConnection("NOT CONNECTED");
                }
            }

            //orb.monitorToORB.mode = 0;

            try
            {
                Thread.sleep(1);
            }
            catch (InterruptedException e)
            {
            }
        }
    }

    //---------------------------------------------------------------
    // Motor
    //---------------------------------------------------------------
    //---------------------------------------------------------------
    /**
     * Set motor configuration. Must be called once after connecting to ORB
     * @param id    Motor port identification, use M1 to M4
     * @param ticsPerRotation   Encoder tics per rotation (Makeblock: use 144)
     * @param acc   Motor acceleration (Makeblock: use 50)
     * @param Kp    Motor controler PID (Makeblock: use 50)
     * @param Ki    Motor controler PID (Makeblock: use 30)
     */
    public void  configMotor( int  id,
                                       int   ticsPerRotation,
                                       int   acc,
                                       int   Kp,
                                       int   Ki )
    {
        configToORB.configMotor(id,ticsPerRotation,acc,Kp,Ki);
    }

    //---------------------------------------------------------------
    /**
     * Set motor.
     * @param id Motor port identification, use M1 to M4
     * @param mode Run mode, use POWER_MODE, BRAKE_MODE, SPEED_MODE or MOVETO_MODE
     * @param speed Motor speed, 1/1000 rotation per second (SPEED_MODE) or 1/1000 of max voltage (POWER_MODE)
     * @param pos Motor position, 1/1000 rotation (MOVETO_MODE only)
     */
    public void setMotor( int id,
                          int mode,
                          int speed,
                          int pos )
    {
        propToORB.setMotor(id, mode, speed, pos);
    }

    //---------------------------------------------------------------
    public short getMotorPwr( byte id )
    {
        return( propFromORB.getMotorPwr(id) );
    }

    //---------------------------------------------------------------
    public short getMotorSpeed( byte id )
    {
        return( propFromORB.getMotorSpeed(id) );
    }

    //---------------------------------------------------------------
    public int getMotorPos( byte id )
    {
        return( propFromORB.getMotorPos(id) );
    }

    //---------------------------------------------------------------
    // ModellServo
    //---------------------------------------------------------------
    //---------------------------------------------------------------
    public void setModelServo( byte id,
                                        int  speed,
                                        int  angle )
    {
        propToORB.setModelServo(id,speed,angle);
    }

    //---------------------------------------------------------------
    // Sensor
    //---------------------------------------------------------------
    public void configSensor( byte id,
                                       byte type,
                                       byte mode,
                                       byte option )
    {
        configToORB.configSensor(id,type,mode,option);
    }

    //---------------------------------------------------------------
    public int getSensorValue( byte id )
    {
        return( propFromORB.getSensorValue(id) );
    }

    //---------------------------------------------------------------
    public int getSensorValueAnalog( byte id, byte ch )
    {
        return( propFromORB.getSensorValueAnalog(id, ch));
    }

    //---------------------------------------------------------------
    public boolean getSensorDigital( byte id )
    {
        return( propFromORB.getSensorDigital(id));
    }

    //---------------------------------------------------------------
    // Miscellaneous
    //---------------------------------------------------------------
    //---------------------------------------------------------------
    public float getVcc()
    {
        return( (float)0.1*propFromORB.getVcc());
    }

    //-----------------------------------------------------------------
}
