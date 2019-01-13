package com.example.xhp.mylive.live;

/**
 * Created by xhp on 2017/12/24.
 */

public class VideoParam {
    private int width;
    private int height;
    private int degree;
    private int cameraId;
    //码率480kbps
    private int bitrate=480000;
    //帧率默认25帧/s
    private int fps=25;

    public int getDegree() {
        return degree;
    }

    public void setDegree(int degree) {
        this.degree = degree;
    }

    public int getBitrate() {
        return bitrate;
    }

    public void setBitrate(int bitrate) {
        this.bitrate = bitrate;
    }

    public int getFps() {
        return fps;
    }

    public void setFps(int fps) {
        this.fps = fps;
    }

    public VideoParam(int width, int height, int cameraId) {
        this.width = width;
        this.height = height;
        this.cameraId = cameraId;
    }

    public int getWidth() {
        return width;
    }

    public void setWidth(int width) {
        this.width = width;
    }

    public int getHeight() {
        return height;
    }

    public void setHeight(int height) {
        this.height = height;
    }

    public int getCameraId() {
        return cameraId;
    }

    public void setCameraId(int cameraId) {
        this.cameraId = cameraId;
    }
}
