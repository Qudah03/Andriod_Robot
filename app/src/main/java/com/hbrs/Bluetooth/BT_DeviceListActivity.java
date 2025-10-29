//*******************************************************************
/*!
\file   BT_DeviceListActivity.java
\author Thomas Breuer
\date   21.02.2019
\brief
*/
package com.hbrs.Bluetooth;

import android.Manifest;
import android.annotation.SuppressLint;
import android.app.Activity;
import android.app.ActivityManager;
import android.bluetooth.BluetoothAdapter;
import android.bluetooth.BluetoothDevice;
import android.content.Intent;
import android.content.pm.PackageManager;
import android.os.Bundle;
import android.util.Log;
import android.view.View;
import android.widget.AdapterView;
import android.widget.ArrayAdapter;
import android.widget.ListView;
import android.widget.Toast;

import androidx.appcompat.app.AppCompatActivity;
import androidx.core.app.ActivityCompat;

import com.hbrs.R;

import java.util.ArrayList;
import java.util.Set;

//*******************************************************************
public class BT_DeviceListActivity extends Activity
{
    @SuppressLint("StaticFieldLeak")
    static Activity parent;
    public static final int RESULT_OK = Activity.RESULT_OK;
    public static final int RESULT_CANCELED = Activity.RESULT_CANCELED;
    public static final int RESULT_NOPERMISSION = Activity.RESULT_FIRST_USER;

    public static void  start(Activity _parent, int requestCode)
    {
        parent = _parent;



        Intent serverIntent = new Intent(parent, BT_DeviceListActivity.class);
        parent.startActivityForResult(serverIntent, requestCode);
    }

    public static BluetoothDevice getDeviceFromIntent(Intent data)
    {

        BluetoothDevice BT_Device;
        String addr = data.getExtras().getString(BT_DeviceListActivity.EXTRA_DEVICE_ADDRESS);

        if (addr.length() > 0) {
            BT_Device = BluetoothAdapter.getDefaultAdapter().getRemoteDevice(addr);
            //orb.openBT(BT_Device);
            return( BT_Device );
        }

        return( null );
    }

    //---------------------------------------------------------------
    @Override
    protected void onCreate(Bundle savedInstanceState)
    {
        super.onCreate(savedInstanceState);

        if (ActivityCompat.checkSelfPermission(parent, Manifest.permission.BLUETOOTH_CONNECT) != PackageManager.PERMISSION_GRANTED) {


            // Create the result Intent and include the MAC address
            Intent intent = new Intent();
            intent.putExtra(EXTRA_DEVICE_ADDRESS, "");

            // Set result and finish this Activity
            setResult(RESULT_NOPERMISSION, intent);
            finish();
            return;
        }
        setContentView(R.layout.device_list);
        createDeviceList();
    }

    //---------------------------------------------------------------
    @Override
    protected void onDestroy()
    {
        super.onDestroy();
    }

    //---------------------------------------------------------------
    class myOnItemClickListener implements AdapterView.OnItemClickListener
    {
        //-----------------------------------------------------------
        @Override
        public void onItemClick(AdapterView<?> arg0, View arg1, int pos, long id)
        {
            //setStatus("onItemClick:"+pos);

            BT_Device = (BluetoothDevice)BT_PairedDevices.toArray()[pos];
            //close();
            //open( BT_Device);


            // Create the result Intent and include the MAC address
            Intent intent = new Intent();
            intent.putExtra(EXTRA_DEVICE_ADDRESS, BT_Device.getAddress());

            // Set result and finish this Activity
            setResult(RESULT_OK, intent);
            finish();
        }
    }



    //---------------------------------------------------------------
    public void createDeviceList()
    {
        BT_Adapter       = BluetoothAdapter.getDefaultAdapter();
        if (ActivityCompat.checkSelfPermission(this, Manifest.permission.BLUETOOTH_CONNECT) != PackageManager.PERMISSION_GRANTED) {
            // TODO: Consider calling
            //    ActivityCompat#requestPermissions
            // here to request the missing permissions, and then overriding
            //   public void onRequestPermissionsResult(int requestCode, String[] permissions,
            //                                          int[] grantResults)
            // to handle the case where the user grants the permission. See the documentation
            // for ActivityCompat#requestPermissions for more details.
            return;
        }
        if (ActivityCompat.checkSelfPermission(this, Manifest.permission.BLUETOOTH_CONNECT) != PackageManager.PERMISSION_GRANTED) {
            // TODO: Consider calling
            //    ActivityCompat#requestPermissions
            // here to request the missing permissions, and then overriding
            //   public void onRequestPermissionsResult(int requestCode, String[] permissions,
            //                                          int[] grantResults)
            // to handle the case where the user grants the permission. See the documentation
            // for ActivityCompat#requestPermissions for more details.
            return;
        }
        BT_PairedDevices = BT_Adapter.getBondedDevices();

        ArrayList<String> list = new ArrayList<String>();

        for(BluetoothDevice BT_Device : BT_PairedDevices)
        {
            if (ActivityCompat.checkSelfPermission(this, Manifest.permission.BLUETOOTH_CONNECT) != PackageManager.PERMISSION_GRANTED) {
                // TODO: Consider calling
                //    ActivityCompat#requestPermissions
                // here to request the missing permissions, and then overriding
                //   public void onRequestPermissionsResult(int requestCode, String[] permissions,
                //                                          int[] grantResults)
                // to handle the case where the user grants the permission. See the documentation
                // for ActivityCompat#requestPermissions for more details.
                return;
            }
            list.add(new String( BT_Device.getName()));
        }

        final ArrayAdapter<String> adapter
                = new ArrayAdapter<String>(this,
                android.R.layout.simple_list_item_1,
                list);

        ListView lv = (ListView)findViewById(R.id.paired_devices);
        lv.setAdapter(adapter);
        lv.setOnItemClickListener( new myOnItemClickListener() );
    }

    BluetoothAdapter     BT_Adapter;
    Set<BluetoothDevice> BT_PairedDevices;
    BluetoothDevice      BT_Device;

    public static String EXTRA_DEVICE_ADDRESS = "device_address";
}