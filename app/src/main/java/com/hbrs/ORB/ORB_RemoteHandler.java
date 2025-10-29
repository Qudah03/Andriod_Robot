//*******************************************************************
/*!
\file   ORB_Remote_Handler.java
\author Thomas Breuer
\date   21.02.2019
\brief
*/

//*******************************************************************
package com.hbrs.ORB;

//*******************************************************************

import android.util.Log;

import java.nio.ByteBuffer;


//*******************************************************************
public class ORB_RemoteHandler
{
    //---------------------------------------------------------------
    public cConfigToORB    configToORB;
    public cPropToORB      propToORB;
    public cPropFromORB   propFromORB;
    public cMonitorToORB      monitorToORB;
    public cMonitorFromORB    monitorFromORB;
    public cSettingsToORB  settingsToORB;
    public cSettingsFromORB   settingsFromORB;

    private   ORB_Remote orbRemote;

    //---------------------------------------------------------------
    protected  ORB_RemoteHandler()
    {
        configToORB     = new cConfigToORB();
        propToORB       = new cPropToORB();
        propFromORB     = new cPropFromORB();
        monitorToORB    = new cMonitorToORB();
        monitorFromORB  = new cMonitorFromORB();
        settingsToORB   = new cSettingsToORB();
        settingsFromORB = new cSettingsFromORB();


        orbRemote = null;
    }

    //---------------------------------------------------------------
    public void init(  )
    {
    }

    //---------------------------------------------------------------
    public void setORB_Remote( ORB_Remote orbRemote )
    {
        this.orbRemote = orbRemote;
    }

    //---------------------------------------------------------------
    public void process( ByteBuffer data )
    {

        propFromORB.get( data );
        monitorFromORB.get( data );
        if( settingsFromORB.get( data ) )
            settingsToORB.isNew = false;

    }

    //---------------------------------------------------------------
    public int fill(  ByteBuffer data )
    {
        int size = 0;
        if( settingsToORB.isNew )
        {
            size = settingsToORB.fill(data);
            //settingsToORB.isNew = false;
        }
        else if( configToORB.isNew ) {
            size = configToORB.fill(data);
            configToORB.isNew = false;
        }
        else if( propToORB.isNew ) {
            Log.d("ORB","propToORB");
            size = propToORB.fill(data);
            propToORB.isNew = false;
        }
        else
        {
            size = monitorToORB.fill( data );
        }
        return( size );
    }

    //---------------------------------------------------------------
    public boolean update()
    {
        if( orbRemote != null )
        {
            return( orbRemote.update() );
        }
        return( false );
    }
} // end of class
