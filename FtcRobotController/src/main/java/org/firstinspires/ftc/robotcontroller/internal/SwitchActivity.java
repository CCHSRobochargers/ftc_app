
package org.firstinspires.ftc.robotcontroller.internal;

import android.app.Activity;
import android.content.Context;
import android.content.Intent;
import android.os.Bundle;
import android.util.Log;
import android.view.View;
import android.widget.SeekBar;
import android.widget.TextView;
import android.widget.ToggleButton;

import com.qualcomm.ftcrobotcontroller.R;

public class SwitchActivity extends Activity {
    ToggleButton cbToggle;
    ToggleButton tileToggle;
    TextView cbText;
    TextView tileText;
    SeekBar delay;

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
        ToggleButton sNToggle = (ToggleButton) findViewById(R.id.shootNumberToggle);
        tileToggle = (ToggleButton) findViewById(R.id.tileToggle);
        tileText = (TextView) findViewById(R.id.tileText);
        cbToggle = (ToggleButton) findViewById(R.id.capballToggle);
        cbText = (TextView) findViewById(R.id.capballText);
        rBToggle.setChecked(AutoConfig.redAlliance);
        bYNToggle.setChecked(AutoConfig.doBeacon);
        sNToggle.setChecked(AutoConfig.shootTwo);
        tileToggle.setChecked(AutoConfig.vortexTile);
        cbToggle.setChecked(AutoConfig.hitCapBall);
        delay = (SeekBar) findViewById(R.id.delay);
        delay.setMax(20);
        delay.setProgress(AutoConfig.delay);
        if (AutoConfig.doBeacon) {
            cbToggle.setVisibility(View.INVISIBLE);
            cbText.setVisibility(View.INVISIBLE);
            tileToggle.setVisibility(View.INVISIBLE);
            tileText.setVisibility(View.INVISIBLE);
            delay.setVisibility(View.INVISIBLE);
        } else {
            cbToggle.setVisibility(View.VISIBLE);
            cbText.setVisibility(View.VISIBLE);
            tileToggle.setVisibility(View.VISIBLE);
            tileText.setVisibility(View.VISIBLE);
            delay.setVisibility(View.VISIBLE);
        }
    }

    public void goBack(View view) {
        AutoConfig.delay = delay.getProgress();
        Intent intent = new Intent(this, FtcRobotControllerActivity.class);
        startActivity(intent);
    }

    public void colorButton(View view) {
        AutoConfig.redAlliance = !AutoConfig.redAlliance;
        Log.i("Red Alliance", String.valueOf(AutoConfig.redAlliance));
}

    public void beaconButton(View view) {
        AutoConfig.doBeacon = !AutoConfig.doBeacon;
        if (AutoConfig.doBeacon) {
            cbToggle.setVisibility(View.INVISIBLE);
            cbText.setVisibility(View.INVISIBLE);
            tileToggle.setVisibility(View.INVISIBLE);
            tileText.setVisibility(View.INVISIBLE);
            delay.setVisibility(View.INVISIBLE);
        } else {
            cbToggle.setVisibility(View.VISIBLE);
            cbText.setVisibility(View.VISIBLE);
            tileToggle.setVisibility(View.VISIBLE);
            tileText.setVisibility(View.VISIBLE);
            delay.setVisibility(View.VISIBLE);
        }
        Log.i("Do Beacon", String.valueOf(AutoConfig.doBeacon));
    }

    public void shootNumberButton(View view) {
        AutoConfig.shootTwo = !AutoConfig.shootTwo;
        Log.i("Shoot #", String.valueOf(AutoConfig.shootTwo ? 2 : 1));
    }

    public void tileButton(View view) {
        AutoConfig.vortexTile = !AutoConfig.vortexTile;
        Log.i("Tile", AutoConfig.vortexTile ? "Vortex" : "Corner");
    }

    public void capballButton(View view) {
        AutoConfig.hitCapBall = !AutoConfig.hitCapBall;
        Log.i("Hit Cap Ball", String .valueOf(AutoConfig.hitCapBall));
    }
}
