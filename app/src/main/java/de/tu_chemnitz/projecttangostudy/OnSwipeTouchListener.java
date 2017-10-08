package de.tu_chemnitz.projecttangostudy;

/**
 * Created by Halabi on 22.05.2016.
 */
import android.content.Context;
import android.util.Log;
import android.view.GestureDetector;
import android.view.GestureDetector.SimpleOnGestureListener;
import android.view.MotionEvent;
import android.view.View;
import android.view.View.OnTouchListener;

public abstract class OnSwipeTouchListener implements OnTouchListener {

    private final GestureDetector gdt = new GestureDetector(this.context, new GestureListener());

    @Override
    public boolean onTouch(final View v, final MotionEvent event) {
        return gdt.onTouchEvent(event);
    }

    private Context context;
    private static final String tag = "MyActivity";

    private final class GestureListener extends SimpleOnGestureListener {

        private static final int SWIPE_MIN_DISTANCE = 60;
        private static final int SWIPE_THRESHOLD_VELOCITY = 100;


        public boolean onDown(MotionEvent evt){
            return true;
        }


        @Override
        public boolean onSingleTapConfirmed(MotionEvent e) {
            click();
            return true;
        }


        @Override
        public boolean onFling(MotionEvent e1, MotionEvent e2, float velocityX, float velocityY) {
            Log.d(tag , "fling");
            if(e1.getX() - e2.getX() > SWIPE_MIN_DISTANCE && Math.abs(velocityX) > SWIPE_THRESHOLD_VELOCITY) {
                onRightToLeft();
                return true;
            } else if (e2.getX() - e1.getX() > SWIPE_MIN_DISTANCE && Math.abs(velocityX) > SWIPE_THRESHOLD_VELOCITY) {
                onLeftToRight();
                return true;
            }
            if(e1.getY() - e2.getY() > SWIPE_MIN_DISTANCE && Math.abs(velocityY) > SWIPE_THRESHOLD_VELOCITY) {
                onBottomToTop();
                return true;
            } else if (e2.getY() - e1.getY() > SWIPE_MIN_DISTANCE && Math.abs(velocityY) > SWIPE_THRESHOLD_VELOCITY) {
                onTopToBottom();
                return true;
            }
            return true;
        }
    }



    public abstract void onRightToLeft();

    public abstract void onLeftToRight();

    public abstract void onBottomToTop();

    public abstract void onTopToBottom();

    public abstract void click();

}