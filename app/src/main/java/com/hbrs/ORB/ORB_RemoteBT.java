package com.hbrs.ORB;

import android.app.Activity;
import android.bluetooth.BluetoothDevice;
import android.bluetooth.BluetoothSocket;
import android.util.Log;

import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.nio.ByteBuffer;
import java.util.ArrayList;
import java.util.UUID;

//*******************************************************************
public class ORB_RemoteBT extends ORB_Remote
{
    //---------------------------------------------------------------
    public ORB_RemoteBT(ORB_RemoteHandler handler)
    {
        super(handler);
        bufferIN  = ByteBuffer.allocate(256);
        bufferOUT = ByteBuffer.allocate(256);
        isConnected = false;
    }

    //---------------------------------------------------------------
    public void init()
    {
    }

    //---------------------------------------------------------------
    public void open( BluetoothDevice BT_Device )
    {
        close();

        try
        {
            BT_Socket  = BT_Device.createRfcommSocketToServiceRecord(MY_UUID);
            BT_Socket.connect();

            BT_OutStream = BT_Socket.getOutputStream();
            BT_InStream  = BT_Socket.getInputStream();

            isConnected = true;
        }
        catch(IOException e)
        {
            isConnected = false;
        }
    }

    //---------------------------------------------------------------
    public void close()
    {
        isConnected = false;
        try
        {
            if( BT_InStream != null ) {
                BT_InStream.close();
            }
            if( BT_OutStream != null ) {
                BT_OutStream.close();
            }
            if( BT_Socket != null ) {
                BT_Socket.close();
            }
        }
        catch( IOException e )
        {
        }
    }

    //---------------------------------------------------------------
    public boolean isConnected()
    {
        return( isConnected );
    }

    //---------------------------------------------------------------

    //---------------------------------------------------------------
    private void updateOut() {
        if (!isConnected) {
            handler.settingsToORB.isNew = true;
            return;
        }

        short crc;
        int size;

        size = handler.fill(bufferOUT);


        crc = CRC(bufferOUT, 2, size);
        bufferOUT.put(0, (byte) (crc & 0xFF));
        bufferOUT.put(1, (byte) ((crc >> 8) & 0xFF));

        byte data[] = new byte[1024];
        short len = 0;
        short idx = 0;

        data[len++] = (byte) 0xA1; // start

        while (idx < size + 2 && len < 1024-1)
        {
            byte b = bufferOUT.get(idx++);
            switch( b )
            {
                case (byte)0xA0: data[len++] = (byte)0xA0;  data[len++] = (byte)0x00;  break;
                case (byte)0xA1: data[len++] = (byte)0xA0;  data[len++] = (byte)0x01;  break;
                case (byte)0xA2: data[len++] = (byte)0xA0;  data[len++] = (byte)0x02;  break;
                default:         data[len++] = b;                                      break;
            }
        }

        data[len++] = (byte)0xA2; // stop


/*
        data[len++] = (byte)(0x20 | ((bufferOUT.get(idx  )>>4) & 0x0F));
        data[len++] = (byte)(0x30 | ((bufferOUT.get(idx++)) & 0x0F));

        while( idx < size+1)
        {
            data[len++] = (byte)(0x40 | ((bufferOUT.get(idx  )>>4) & 0x0F));
            data[len++] = (byte)(0x50 | ((bufferOUT.get(idx++)   ) & 0x0F));
        }
        data[len++] = (byte)(0x80 | ((bufferOUT.get(idx  )>>4) & 0x0F));
        data[len++] = (byte)(0x90 | ((bufferOUT.get(idx++)   ) & 0x0F));
*/


        try
        {
            BT_OutStream.write(data,0,len);
            BT_OutStream.flush();
        }
        catch( IOException e )
        {
        }
    }

    //---------------------------------------------------------------
    private boolean  updateIn()
    {
        if(!isConnected)
        {
            return(false);
        }

        int r;
        try
        {
            while( !ready && BT_InStream.available() > 0 && (r = BT_InStream.read()) >= 0 )
            {
                switch( r )
                {
                    case 0xA1: // start
                        pos = 0;
                        flag = false;
                        break;

                    case 0xA2: // STOP
                        ready = true;
                        flag = false;
                        break;

                    case 0xA0: //
                        flag = true;
                        break;

                    default:
                        if( flag )
                        {
                            r += 0xA0;
                        }
                        bufferIN.put( pos++, (byte)r );
                        flag = false;
                        break;
                }
            }
        }
        catch (IOException e)
        {
            Log.e("ORB_BT", "error read ");
        }

        if( pos >= 255 )
        {
            ready = false;
            pos = 0;
            flag = false;
        }
        if( ready )
        {
            handler.process( bufferIN );
            ready = false;
            pos = 0;
            flag = false;
            return (true);
        }
        return( false );
    }

    //---------------------------------------------------------------
    public boolean update()
    {
        boolean ret = false;

        if( updateIn() )
        {
            ret = true;
            updateCnt = UPDATE_CNT_MAX;
        }

        if( updateIn() )
        {
            ret = true;
            updateCnt = UPDATE_CNT_MAX;
        }

        if( updateCnt++ >= UPDATE_CNT_MAX )
        {
            updateOut();
            updateCnt = 0;
        }
        return( ret );
    }

    //---------------------------------------------------------------
    //---------------------------------------------------------------
    private BluetoothSocket BT_Socket;
    private OutputStream    BT_OutStream;
    private InputStream     BT_InStream;
    private boolean         isConnected = false;

    //---------------------------------------------------------------
    private ByteBuffer bufferIN;
    private ByteBuffer bufferOUT;

    //---------------------------------------------------------------
    private boolean ready     = false;
    private int     pos       = 0;
    private byte    temp      = 0;
    private int     updateCnt = 0;

    private boolean flag = false;

    //---------------------------------------------------------------
    private static final UUID MY_UUID = UUID.fromString("00001101-0000-1000-8000-00805F9B34FB");
    private static final int  UPDATE_CNT_MAX = 1;
}
