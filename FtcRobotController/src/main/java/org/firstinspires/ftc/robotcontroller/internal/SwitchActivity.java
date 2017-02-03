
package org.firstinspires.ftc.robotcontroller.internal;

import android.app.Activity;
import android.content.Context;
import android.content.Intent;
import android.os.Bundle;
import android.util.Log;
import android.view.View;
import android.widget.ToggleButton;

import com.qualcomm.ftcrobotcontroller.R;

public class SwitchActivity extends Activity {

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_switch);
    }

    @Override
    protected void onStart() {
        super.onStart();
        ToggleButton rBToggle = (ToggleButton) findViewById(R.id.rBToggle);
        ToggleButton bYNToggle = (ToggleButton) findViewById(R.id.bYNToggle);
        rBToggle.setChecked(AutoConfig.redAlliance);
        bYNToggle.setChecked(AutoConfig.doBeacon);
    }

    public void goBack(View view) {
        Intent intent = new Intent(this, FtcRobotControllerActivity.class);
        startActivity(intent);
    }

    public void colorButton(View view) {
        AutoConfig.redAlliance = !AutoConfig.redAlliance;
        Log.i("Red Alliance", String.valueOf(AutoConfig.redAlliance));
}

    public void beaconButton(View view) {
        AutoConfig.doBeacon = !AutoConfig.doBeacon;
        Log.i("Do Beacon", String.valueOf(AutoConfig.doBeacon));
    }
}
