package com.example.xhp.mylive.live;

import android.graphics.ImageFormat;
import android.hardware.Camera;
import android.util.Log;
import android.view.SurfaceHolder;

import com.example.xhp.mylive.jni.LiveNative;

import java.io.IOException;

/**
 * Created by xhp on 2017/12/24.
 */

public class VideoPusher implements Pusher, SurfaceHolder.Callback, Camera.PreviewCallback {
    private static final String TAG = VideoPusher.class.getSimpleName();
    private SurfaceHolder surfaceHolder;
    private Camera camera;
    private VideoParam videoParam;
    private byte[] buffers;
    private LiveNative liveNative;
    //private Context context;
    private boolean isPushing = false;

    public VideoPusher(SurfaceHolder surfaceHolder, LiveNative liveNative, VideoParam videoParam) {
        this.surfaceHolder = surfaceHolder;
        this.liveNative = liveNative;
        //this.context = context;
        this.videoParam = videoParam;

        surfaceHolder.addCallback(this);
        liveNative.setVideoOptions(videoParam.getWidth(), videoParam.getHeight(), videoParam.getBitrate(), videoParam.getFps());
    }

    @Override
    public void startPush() {
        isPushing = true;
    }

    @Override
    public void stopPush() {
        isPushing = false;
    }

    @Override
    public void surfaceCreated(SurfaceHolder holder) {
        startPreview();
    }

    @Override
    public void surfaceChanged(SurfaceHolder holder, int format, int width, int height) {
        Log.d(TAG, "surfaceChanged format=" + format + ",width=" + width + ",height=" + height);
    }

    @Override
    public void surfaceDestroyed(SurfaceHolder holder) {
        stopPreview();

    }

    private void startPreview() {
        camera = Camera.open(videoParam.getCameraId());
        try {
            Camera.Parameters parameters = camera.getParameters();
            parameters.setPreviewFormat(ImageFormat.NV21);
            parameters.setPreviewSize(videoParam.getWidth(), videoParam.getHeight());
            parameters.setRotation(videoParam.getDegree());
            camera.setParameters(parameters);
//            List<Camera.Size> sizeList = parameters.getSupportedPreviewSizes();
//            for (Camera.Size size : sizeList) {
//                Log.d(TAG, "width=" + size.width + ",height=" + size.height);
//            }
            camera.setPreviewDisplay(surfaceHolder);
//            if (context.getResources().getConfiguration().orientation == Configuration.ORIENTATION_PORTRAIT) {
//                camera.setDisplayOrientation(90);
//            } else {
//                camera.setDisplayOrientation(0);
//            }
            camera.setDisplayOrientation(videoParam.getDegree());
            buffers = new byte[videoParam.getHeight() * videoParam.getWidth() * 4];
            camera.addCallbackBuffer(buffers);
            camera.setPreviewCallbackWithBuffer(this);
            camera.startPreview();
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    private void stopPreview() {
        if (camera != null) {
            camera.stopPreview();
            camera.release();
            camera = null;
        }
    }

    @Override
    public void onPreviewFrame(byte[] data, Camera camera) {

        camera.addCallbackBuffer(buffers);
        if (isPushing) {
            //推送数据到服务器
            Log.d(TAG, "onPreviewFrame");
            liveNative.sendVideoData(data);
        }
    }
}
