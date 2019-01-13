package com.example.xhp.mylive.live;

import android.media.AudioFormat;
import android.media.AudioRecord;
import android.media.MediaRecorder;
import android.util.Log;

import com.example.xhp.mylive.jni.LiveNative;

import java.util.concurrent.Executors;

/**
 * Created by xhp on 2017/12/24.
 */

public class AudioPusher implements Pusher {
    private static final String TAG=AudioPusher.class.getSimpleName();
    private boolean isPushing=false;
    private LiveNative liveNative;
    private AudioParam audioParam;
    private AudioRecord audioRecord;
    private int minBufferSize;
    private AudioRecordTask audioRecordTask;

    public AudioPusher(LiveNative liveNative) {
        this.liveNative = liveNative;
        audioParam = new AudioParam();
        int channelConfig = audioParam.getChannel() == 1 ? AudioFormat.CHANNEL_IN_MONO : AudioFormat.CHANNEL_IN_STEREO;
        int audioFormat = AudioFormat.ENCODING_PCM_16BIT;
        minBufferSize = AudioRecord.getMinBufferSize(audioParam.getSampleRateInHz(), channelConfig, audioFormat);
        audioRecord = new AudioRecord(MediaRecorder.AudioSource.MIC, audioParam.getSampleRateInHz(),
                channelConfig, audioFormat, minBufferSize);
        audioRecordTask=new AudioRecordTask();
        liveNative.setAudioOptions(audioParam.getSampleRateInHz(),audioParam.getChannel());
    }

    @Override
    public void startPush() {
        isPushing=true;
        Executors.newSingleThreadExecutor().execute(audioRecordTask);
    }

    @Override
    public void stopPush() {
        isPushing=false;
        audioRecord.stop();
    }

    public void release(){
        if(audioRecord!=null){
            audioRecord.release();
        }
    }

    class AudioRecordTask implements Runnable{

        @Override
        public void run() {
            audioRecord.startRecording();
            while (isPushing){
                Log.d(TAG,"AudioRecordTask read");
                byte[] buffer=new byte[minBufferSize];
                int len=audioRecord.read(buffer,0,buffer.length);
                if(len>0){
                    liveNative.sendAudio(buffer,len);
                }
            }
        }
    }
}
