package de.tu_chemnitz.projecttangostudy;

import android.app.Activity;
import android.content.Context;
import android.media.AudioManager;
import android.media.MediaPlayer;
import android.os.Vibrator;
import android.util.Log;

/**
 * Created by Halabi on 13.04.2017.
 */

public class Android_Functionality_Class {

    public static void make_vibration(int i, Context c) {

        //Vibrate
        Vibrator v = (Vibrator) c.getSystemService(Context.VIBRATOR_SERVICE);

        if (v.hasVibrator()) {
            v.vibrate(i);
        } else {
            Log.v("Vibration", "NOT SUPPORTED!");
        }

        Thread thread = new Thread() { //remove plitz
            @Override
            public void run() {
                try {
                    sleep(1000);
                    FloorPlanNavigator.dev_allows_vibration_yet = true;
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }
        };

        thread.start();

    }

    public static void make_robot_sound(Activity activity) {
        AudioManager am = (AudioManager)activity.getSystemService(Context.AUDIO_SERVICE);

        if(am.getRingerMode() != AudioManager.RINGER_MODE_SILENT && am.getRingerMode() != AudioManager.RINGER_MODE_VIBRATE) {
            MediaPlayer chomp_sound = MediaPlayer.create(activity, R.raw.robot);
            chomp_sound.start();
            chomp_sound.setOnCompletionListener(new MediaPlayer.OnCompletionListener() {
                @Override
                public void onCompletion(MediaPlayer mp) {
                    if(mp != null)
                        mp.release();
                }
            });
        }

    }
}
