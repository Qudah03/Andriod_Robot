package com.hbrs.ORB;

import android.content.Intent;
import android.hardware.usb.UsbConstants;
import android.hardware.usb.UsbDevice;
import android.hardware.usb.UsbDeviceConnection;
import android.hardware.usb.UsbEndpoint;
import android.hardware.usb.UsbInterface;
import android.hardware.usb.UsbManager;
import android.hardware.usb.UsbRequest;
import android.util.Log;

import java.nio.ByteBuffer;
import java.util.HashMap;
import java.util.Iterator;

//*******************************************************************
public class ORB_RemoteUSB extends ORB_Remote
{
    //---------------------------------------------------------------
    public ORB_RemoteUSB(ORB_RemoteHandler handler)
    {
        super(handler);
        bufferIN = ByteBuffer.allocate(128);
        bufferOUT = ByteBuffer.allocate(128);
    }

    //---------------------------------------------------------------
    public void init(UsbManager _mUsbManager)
    {
        this.mUsbManager = _mUsbManager;
    }

    //---------------------------------------------------------------
    public void open(Intent intent)
    {
        String action = intent.getAction();

        UsbDevice device = (UsbDevice) intent.getParcelableExtra(UsbManager.EXTRA_DEVICE);

        if( UsbManager.ACTION_USB_DEVICE_ATTACHED.equals(action) )
        {
            setDevice(device);
        }
        else if( UsbManager.ACTION_USB_DEVICE_DETACHED.equals(action) )
        {
            if( mDevice != null && mDevice.equals(device) )
            {
                setDevice(null);
            }
        }
        else
        {
            HashMap<String, UsbDevice> map = mUsbManager.getDeviceList();

            Iterator<UsbDevice> it = map.values().iterator();

            while (it.hasNext())
            {
                device = it.next();
                setDevice(device);
            }
        }
    }

    //---------------------------------------------------------------
    public void close()
    {
        usbConnected=false;
    }

    //---------------------------------------------------------------
    public boolean isConnected()
    {
        return( usbConnected );
    }

    //-----------------------------------------------------------------
    private void setDevice(UsbDevice device)
    {
        Log.w(TAG, "setDevice " + device);
        if (device.getInterfaceCount() != 1)
        {
            Log.e(TAG, "could not find interface");
            return;
        }

        UsbInterface intf = device.getInterface(0);
        // device should have one endpoint
        if (intf.getEndpointCount() < 1)
        {
            Log.e(TAG, "could not find endpoint");
            return;
        }

        // endpoint should be of type interrupt
        // todo index is NOT EP address, check mAdress == 129 or epIN.getDirection() == UsbConstants.USB_DIR_IN
        UsbEndpoint epIN = intf.getEndpoint(1);
        if (epIN.getType() != UsbConstants.USB_ENDPOINT_XFER_INT)
        {
            Log.e(TAG, "endpoint is not interrupt type");
            return;
        }

        // todo index is NOT EP address, check mAdress == 1 or epOUT.getDirection() == UsbConstants.USB_DIR_OUT
        UsbEndpoint epOUT = intf.getEndpoint(0);
        if (epOUT.getType() != UsbConstants.USB_ENDPOINT_XFER_INT)
        {
            Log.e(TAG, "endpoint is not interrupt type");
            return;
        }

        mDevice      = device;

        if( device != null )
        {
            UsbDeviceConnection connection = mUsbManager.openDevice(device);
            if (connection != null && connection.claimInterface(intf, true))
            {
                Log.e(TAG, "open SUCCESS");
                mConnection = connection;
                usbConnected = true;

                requestIN = new UsbRequest();
                requestIN.initialize(mConnection, epIN);

                requestOUT = new UsbRequest();
                requestOUT.initialize(mConnection, epOUT);
            }
            else
            {
                Log.d(TAG, "open FAIL");
                mConnection = null;
            }
        }
        else
        {
            usbConnected=false;
        }
    }

    //---------------------------------------------------------------

    //---------------------------------------------------------------
    private void updateOut()
    {
        short crc;
        int size;

        size = handler.fill(bufferOUT);

        crc = CRC(bufferOUT, 2, size);

        bufferOUT.put(0, (byte) (crc & 0xFF));
        bufferOUT.put(1, (byte) ((crc >> 8) & 0xFF));

        requestOUT.queue(bufferOUT, 64);

        if (mConnection.requestWait() == requestOUT) // wait for status event
        {
            //Log.i(TAG, "CONFIG");
        }
    }

    //---------------------------------------------------------------
    private boolean updateIn()
    {
        requestIN.queue(bufferIN, 64+1);  // queue a request on the interrupt endpoint

        if (mConnection.requestWait() == requestIN) // wait for status event
        {
            handler.process( bufferIN );
            return (true);
        }
        return (false);
    }

    //---------------------------------------------------------------
    public boolean update()
    {
        boolean ret = false;

        if( usbConnected && updateIn() )
        {
            ret = true;
            updateCnt = UPDATE_CNT_MAX;

        }
        if( usbConnected && updateIn() )
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
    private ByteBuffer bufferIN;
    private ByteBuffer bufferOUT;

    //---------------------------------------------------------------
    private UsbManager          mUsbManager;
    private UsbDevice           mDevice;
    private UsbDeviceConnection mConnection;

    private UsbRequest          requestIN;
    private UsbRequest          requestOUT;

    //---------------------------------------------------------------
    private boolean usbConnected = false;

    private int     updateCnt = 0;

    private static final String TAG = "ORB_USB";

    private static final int  UPDATE_CNT_MAX = 1;
}
