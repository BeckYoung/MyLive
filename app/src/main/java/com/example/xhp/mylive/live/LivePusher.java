package com.example.xhp.mylive.live;

import android.content.Context;
import android.view.SurfaceHolder;
import android.view.SurfaceView;

import com.example.xhp.mylive.jni.LiveNative;

/**
 * 推流总控制器
 * Created by xhp on 2017/12/24.
 */

public class LivePusher implements Pusher, SurfaceHolder.Callback {
    private String liveUrl="rtmp://47.52.77.225:1935/live/test";
    //private String liveUrl="rtmp://192.168.0.5/xhp/live";
    private SurfaceHolder surfaceHolder;
    private VideoPusher videoPusher;
    private AudioPusher audioPusher;
    private LiveNative liveNative;
    private Context context;

    public LivePusher(SurfaceView surfaceView,VideoParam videoParam) {
        this.surfaceHolder = surfaceView.getHolder();
        this.context=surfaceView.getContext();
        surfaceHolder.addCallback(this);

        liveNative=new LiveNative();
        liveNative.init();

        videoPusher=new VideoPusher(surfaceHolder,liveNative,videoParam);
        audioPusher=new AudioPusher(liveNative);
    }

    @Override
    public void startPush() {
        liveNative.startPush(liveUrl);
        videoPusher.startPush();
        audioPusher.startPush();
    }

    @Override
    public void stopPush() {
        videoPusher.stopPush();
        audioPusher.stopPush();
        liveNative.stopPush();
    }

    @Override
    public void surfaceCreated(SurfaceHolder holder) {

    }

    @Override
    public void surfaceChanged(SurfaceHolder holder, int format, int width, int height) {

    }

    @Override
    public void surfaceDestroyed(SurfaceHolder holder) {
        stopPush();
        audioPusher.release();
    }
}
