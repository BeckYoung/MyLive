package com.example.xhp.mylive.jni;

/**
 * Created by xhp on 2017/12/24.
 */

public class LiveNative {
    public native void init();
    public native void startPush(String url);
    public native void stopPush();
    public native void realse();
    public native void setVideoOptions(int width,int height,int bitrate,int fps);
    public native void sendVideoData(byte[] data);

    /**
     * 设置音频参数
     * @param sampleRateInHz
     * @param channel
     */
    public native void setAudioOptions(int sampleRateInHz, int channel);

    /**
     * 发送音频数据
     * @param data
     * @param len
     */
    public native void sendAudio(byte[] data, int len);

    // Used to load the 'native-lib' library on application startup.
    static {
        System.loadLibrary("native-lib");
    }
}
