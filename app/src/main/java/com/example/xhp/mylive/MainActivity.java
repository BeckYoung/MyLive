package com.example.xhp.mylive;

import android.hardware.Camera;
import android.os.Bundle;
import android.support.v7.app.AppCompatActivity;
import android.view.Surface;
import android.view.SurfaceHolder;
import android.view.SurfaceView;
import android.view.View;
import android.widget.Button;

import com.example.xhp.mylive.live.LivePusher;
import com.example.xhp.mylive.live.VideoParam;

public class MainActivity extends AppCompatActivity implements View.OnClickListener {
    private String liveUrl="rtmp://47.52.77.225/xhp/live";
    private SurfaceView surfaceView;
    private LivePusher livePusher;
    private Button btn_push;
    private VideoParam videoParam;


    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);

        surfaceView=findViewById(R.id.surfaceView);
        btn_push=findViewById(R.id.btn_push);
        btn_push.setOnClickListener(this);

        videoParam = new VideoParam(640, 480, Camera.CameraInfo.CAMERA_FACING_BACK);
        videoParam.setDegree(getDisplayOritation(getDispalyRotation(),videoParam.getCameraId()));
        SurfaceHolder surfaceHolder=surfaceView.getHolder();
        surfaceHolder.setFixedSize(videoParam.getWidth(),videoParam.getHeight());
        surfaceHolder.setKeepScreenOn(true);

//        DisplayMetrics displayMetrics=getResources().getDisplayMetrics();
//        int width=displayMetrics.widthPixels;
//        int height=displayMetrics.heightPixels;
//        int iNewWidth = (int) (height * 3.0 / 4.0);
//        RelativeLayout.LayoutParams layoutParams = new RelativeLayout.LayoutParams(RelativeLayout.LayoutParams.MATCH_PARENT,
//                RelativeLayout.LayoutParams.MATCH_PARENT);
//        int iPos = width - iNewWidth;
//        layoutParams.setMargins(iPos, 0, 0, 0);
//        surfaceView.setLayoutParams(layoutParams);

        livePusher=new LivePusher(surfaceView,videoParam);
        setBtn_pushStatus(btn_push.isSelected());
    }


    @Override
    public void onClick(View v) {
        boolean isSelected=btn_push.isSelected();
        if(isSelected){
            //停止正在的直播
            livePusher.stopPush();
        }else {
            //开始live调用
            livePusher.startPush();
        }
        btn_push.setSelected(!isSelected);
        setBtn_pushStatus(!isSelected);
    }

    private void setBtn_pushStatus(boolean isSelect){
        if(isSelect){
            btn_push.setText("停止直播");
        }else {
            btn_push.setText("开始直播");
        }
    }

    private int getDispalyRotation() {
        int i = getWindowManager().getDefaultDisplay().getRotation();
        switch (i) {
            case Surface.ROTATION_0:
                return 0;
            case Surface.ROTATION_90:
                return 90;
            case Surface.ROTATION_180:
                return 180;
            case Surface.ROTATION_270:
                return 270;
        }
        return 0;
    }

    private int getDisplayOritation(int degrees, int cameraId) {
        Camera.CameraInfo info = new Camera.CameraInfo();
        Camera.getCameraInfo(cameraId, info);
        int result = 0;
        if (info.facing == Camera.CameraInfo.CAMERA_FACING_FRONT) {
            result = (info.orientation + degrees) % 360;
            result = (360 - result) % 360;
        } else {
            result = (info.orientation - degrees + 360) % 360;
        }
        return result;
    }
}
