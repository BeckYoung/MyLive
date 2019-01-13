#include <jni.h>
#include <string>
#include <iostream>
#include "rtmp.h"
#include "log.h"
#include "queue.h"
#include "x264.h"
#include "aacenc_lib.h"

void destoryRtmp();

void add_aac_sequence_header();

void add_264_sequence_header(unsigned char *, unsigned char *, int, int);

void add_264_body(unsigned char *, int);

char *rtmp_path;//流媒体地址
pthread_mutex_t mutex;//线程互斥锁
pthread_cond_t cond;//线程条件变量
unsigned int start_time;//rtmp连接的时间
bool isPushing = false;
RTMP *rtmp;

//YUV个数
int y_len, u_len, v_len;
//x264编码处理器
x264_t *video_encode_handle;
//x264编码输入图像YUV420P
x264_picture_t pic_in;
x264_picture_t pic_out;

//音频处理变量
HANDLE_AACENCODER handle_aacencoder = NULL;
AACENC_InfoStruct pInfo;

void *push_thread(void *arg) {
    LOGD("视频线程启动成功 url=%s", rtmp_path);
    while (isPushing) {
        pthread_mutex_lock(&mutex);
        pthread_cond_wait(&cond, &mutex);
        //取出队列中的RTMPPacket
        RTMPPacket *packet = (RTMPPacket *) queue_get_first();
        if (packet) {
            queue_delete_first(); //移除
            packet->m_nInfoField2 = rtmp->m_stream_id; //RTMP协议，stream_id数据
            int i = RTMP_SendPacket(rtmp, packet, TRUE); //TRUE放入librtmp队列中，并不是立即发送
            if (!i) {
                LOGE("RTMP 断开");
                RTMPPacket_Free(packet);
                pthread_mutex_unlock(&mutex);
                destoryRtmp();
            }
            RTMPPacket_Free(packet);
        }

        pthread_mutex_unlock(&mutex);
    }
    LOGD("线程结束！");
    return 0;
}

void destoryRtmp() {
    LOGI("%s", "释放资源");
    if (rtmp) {
        RTMP_Close(rtmp);
        RTMP_Free(rtmp);
    }
}

void initRtmp() {
    LOGD("初始化rtmp");
    //建立RTMP连接
    rtmp = RTMP_Alloc();
    if (!rtmp) {
        LOGE("rtmp初始化失败");
        return;
    }
    RTMP_Init(rtmp);
    rtmp->Link.timeout = 5; //连接超时的时间
    //设置流媒体地址
    RTMP_SetupURL(rtmp, rtmp_path);
    //发布rtmp数据流
    RTMP_EnableWrite(rtmp);
    //建立连接
    if (!RTMP_Connect(rtmp, NULL)) {
        LOGE("%s", "RTMP 连接失败");
        destoryRtmp();
        return;
    }
    //计时
    start_time = RTMP_GetTime();
    if (!RTMP_ConnectStream(rtmp, 0)) { //连接流
        destoryRtmp();
        return;
    }
    LOGD("连接流媒体服务器成功");
    //发送aac头信息
    add_aac_sequence_header();
}

extern "C"
JNIEXPORT void JNICALL
Java_com_example_xhp_mylive_jni_LiveNative_init(JNIEnv *env, jobject instance) {

    //创建队列
    create_queue();
    LOGD("init success");
}

extern "C"
JNIEXPORT void JNICALL
Java_com_example_xhp_mylive_jni_LiveNative_startPush(JNIEnv *env, jobject instance,
                                                     jstring jstr_url) {
    const char *ctr_url = env->GetStringUTFChars(jstr_url, NULL);
    //复制url_cstr内容到rtmp_path
    int slen = strlen(ctr_url);
    rtmp_path = (char *) malloc(slen + 1);
    memset(rtmp_path, 0, slen + 1);
    memcpy(rtmp_path, ctr_url, slen);
    //初始化线程互斥锁和条件变量
    pthread_mutex_init(&mutex, NULL);
    pthread_cond_init(&cond, NULL);
    isPushing = true;
    if (NULL == rtmp)
        initRtmp();
    //创建消费者线程
    pthread_t pthread_id;
    pthread_create(&pthread_id, NULL, push_thread, NULL);

    env->ReleaseStringUTFChars(jstr_url, ctr_url);
}

extern "C"
JNIEXPORT void JNICALL
Java_com_example_xhp_mylive_jni_LiveNative_stopPush(JNIEnv *env, jobject instance) {
    LOGD("停止直播");
    isPushing = false;
    pthread_cond_signal(&cond);
    pthread_mutex_destroy(&mutex);
    pthread_cond_destroy(&cond);
}

extern "C"
JNIEXPORT void JNICALL
Java_com_example_xhp_mylive_jni_LiveNative_realse(JNIEnv *env, jobject instance) {

    queue_destroy();
    destoryRtmp();

}

extern "C"
JNIEXPORT void JNICALL
Java_com_example_xhp_mylive_jni_LiveNative_setVideoOptions(JNIEnv *env, jobject instance,
                                                           jint width, jint height, jint bitrate,
                                                           jint fps) {

    x264_param_t param;
    //x264_param_default_preset 设置
    x264_param_default_preset(&param, "ultrafast", "zerolatency");
    //编码输入的像素格式YUV420P
    param.i_csp = X264_CSP_I420;
    param.i_width = width;
    param.i_height = height;

    y_len = width * height;
    u_len = y_len / 4;
    v_len = u_len;

    //参数i_rc_method表示码率控制，CQP(恒定质量)，CRF(恒定码率)，ABR(平均码率)
    //恒定码率，会尽量控制在固定码率
    param.rc.i_rc_method = X264_RC_CRF;
    param.rc.i_bitrate = bitrate / 1000; //* 码率(比特率,单位Kbps)
    param.rc.i_vbv_max_bitrate = bitrate / 1000 * 1.2; //瞬时最大码率

    //码率控制不通过timebase和timestamp，而是fps
    param.b_vfr_input = 0;
    param.i_fps_num = fps; //* 帧率分子
    param.i_fps_den = 1; //* 帧率分母
    param.i_timebase_den = param.i_fps_num;
    param.i_timebase_num = param.i_fps_den;
    param.i_threads = 1;//并行编码线程数量，0默认为多线程

    //是否把SPS和PPS放入每一个关键帧
    //SPS Sequence Parameter Set 序列参数集，PPS Picture Parameter Set 图像参数集
    //为了提高图像的纠错能力
    param.b_repeat_headers = 1;
    //设置Level级别
    param.i_level_idc = 51;
    //设置Profile档次
    //baseline级别，没有B帧
    x264_param_apply_profile(&param, "baseline");

    //x264_picture_t（输入图像）初始化
    x264_picture_alloc(&pic_in, param.i_csp, param.i_width, param.i_height);
    pic_in.i_pts = 0;
    //打开编码器
    video_encode_handle = x264_encoder_open(&param);
    if (video_encode_handle) {
        LOGI("打开视频编码器成功...");
    }

}

extern "C"
JNIEXPORT void JNICALL
Java_com_example_xhp_mylive_jni_LiveNative_sendVideoData(JNIEnv *env, jobject instance,
                                                         jbyteArray data_) {
    jbyte *nv21_buffer = env->GetByteArrayElements(data_, NULL);
    uint8_t *u = pic_in.img.plane[1];
    uint8_t *v = pic_in.img.plane[2];
    memcpy(pic_in.img.plane[0], nv21_buffer, y_len);
    for (int i = 0; i < u_len; i++) {
        *(u + i) = *(nv21_buffer + y_len + i * 2 + 1);
        *(v + i) = *(nv21_buffer + y_len + i * 2);
    }

    //h264编码得到NALU数组
    x264_nal_t *nal = NULL; //NAL
    int n_nal = -1; //NALU的个数
    //进行h264编码
    if (x264_encoder_encode(video_encode_handle, &nal, &n_nal, &pic_in, &pic_out) < 0) {
        LOGE("%s", "编码失败");
        return;
    }
    //使用rtmp协议将h264编码的视频数据发送给流媒体服务器
    //帧分为关键帧和普通帧，为了提高画面的纠错率，关键帧应包含SPS和PPS数据
    int sps_len, pps_len;
    unsigned char sps[100];
    unsigned char pps[100];
    memset(sps, 0, 100);
    memset(pps, 0, 100);
    pic_in.i_pts += 1;
    //遍历NALU数组，根据NALU的类型判断
    for (int i = 0; i < n_nal; i++) {
        if (nal[i].i_type == NAL_SPS) {
            //复制SPS数据
            sps_len = nal[i].i_payload - 4;
            memcpy(sps, nal[i].p_payload + 4, sps_len); //不复制四字节起始码
        } else if (nal[i].i_type == NAL_PPS) {
            //复制PPS数据
            pps_len = nal[i].i_payload - 4;
            memcpy(pps, nal[i].p_payload + 4, pps_len); //不复制四字节起始码

            //发送序列信息
            //h264关键帧会包含SPS和PPS数据
            add_264_sequence_header(pps, sps, pps_len, sps_len);

        } else {
            //发送帧信息
            add_264_body(nal[i].p_payload, nal[i].i_payload);
        }

    }

    env->ReleaseByteArrayElements(data_, nv21_buffer, 0);
}

/**
 * 加入RTMPPacket队列，等待发送线程发送
 */
void add_rtmp_packet(RTMPPacket *packet) {
    pthread_mutex_lock(&mutex);
    queue_append_last(packet);
    pthread_cond_signal(&cond);
    pthread_mutex_unlock(&mutex);
}

/**
 * 发送h264 SPS与PPS参数集
 */
void add_264_sequence_header(unsigned char *pps, unsigned char *sps, int pps_len, int sps_len) {
    int body_size = 16 + sps_len + pps_len; //按照H264标准配置SPS和PPS，共使用了16字节
    RTMPPacket *packet = (RTMPPacket *) malloc(sizeof(RTMPPacket));
    //RTMPPacket初始化
    RTMPPacket_Alloc(packet, body_size);
    RTMPPacket_Reset(packet);

    unsigned char *body = (unsigned char *) packet->m_body;
    int i = 0;
    //二进制表示：00010111
    body[i++] = 0x17;//VideoHeaderTag:FrameType(1=key frame)+CodecID(7=AVC)
    body[i++] = 0x00;//AVCPacketType = 0表示设置AVCDecoderConfigurationRecord
    //composition time 0x000000 24bit ?
    body[i++] = 0x00;
    body[i++] = 0x00;
    body[i++] = 0x00;

    /*AVCDecoderConfigurationRecord*/
    body[i++] = 0x01;//configurationVersion，版本为1
    body[i++] = sps[1];//AVCProfileIndication
    body[i++] = sps[2];//profile_compatibility
    body[i++] = sps[3];//AVCLevelIndication
    //?
    body[i++] = 0xFF;//lengthSizeMinusOne,H264 视频中 NALU的长度，计算方法是 1 + (lengthSizeMinusOne & 3),实际测试时发现总为FF，计算结果为4.

    /*sps*/
    body[i++] = 0xE1;//numOfSequenceParameterSets:SPS的个数，计算方法是 numOfSequenceParameterSets & 0x1F,实际测试时发现总为E1，计算结果为1.
    body[i++] = (sps_len >> 8) & 0xff;//sequenceParameterSetLength:SPS的长度
    body[i++] = sps_len & 0xff;//sequenceParameterSetNALUnits
    memcpy(&body[i], sps, sps_len);
    i += sps_len;

    /*pps*/
    body[i++] = 0x01;//numOfPictureParameterSets:PPS 的个数,计算方法是 numOfPictureParameterSets & 0x1F,实际测试时发现总为E1，计算结果为1.
    body[i++] = (pps_len >> 8) & 0xff;//pictureParameterSetLength:PPS的长度
    body[i++] = (pps_len) & 0xff;//PPS
    memcpy(&body[i], pps, pps_len);
    i += pps_len;

    //Message Type，RTMP_PACKET_TYPE_VIDEO：0x09
    packet->m_packetType = RTMP_PACKET_TYPE_VIDEO;
    //Payload Length
    packet->m_nBodySize = body_size;
    //Time Stamp：4字节
    //记录了每一个tag相对于第一个tag（File Header）的相对时间。
    //以毫秒为单位。而File Header的time stamp永远为0。
    packet->m_nTimeStamp = 0;
    packet->m_hasAbsTimestamp = 0;
    packet->m_nChannel = 0x04; //Channel ID，Audio和Vidio通道
    packet->m_headerType = RTMP_PACKET_SIZE_MEDIUM; //?
    //将RTMPPacket加入队列
    add_rtmp_packet(packet);

}

/**
 * 发送h264帧信息
 */
void add_264_body(unsigned char *buf, int len) {
    //去掉起始码(界定符)
    if (buf[2] == 0x00) {  //00 00 00 01
        buf += 4;
        len -= 4;
    } else if (buf[2] == 0x01) { // 00 00 01
        buf += 3;
        len -= 3;
    }
    int body_size = len + 9;
    RTMPPacket *packet = (RTMPPacket *) malloc(sizeof(RTMPPacket));
    RTMPPacket_Alloc(packet, body_size);

    unsigned char *body = (unsigned char *) packet->m_body;
    //当NAL头信息中，type（5位）等于5，说明这是关键帧NAL单元
    //buf[0] NAL Header与运算，获取type，根据type判断关键帧和普通帧
    //00000101 & 00011111(0x1f) = 00000101
    int type = buf[0] & 0x1f;
    //Inter Frame 帧间压缩
    body[0] = 0x27;//VideoHeaderTag:FrameType(2=Inter Frame)+CodecID(7=AVC)
    //IDR I帧图像
    if (type == NAL_SLICE_IDR) {
        body[0] = 0x17;//VideoHeaderTag:FrameType(1=key frame)+CodecID(7=AVC)
    }
    //AVCPacketType = 1
    body[1] = 0x01; /*nal unit,NALUs（AVCPacketType == 1)*/
    body[2] = 0x00; //composition time 0x000000 24bit
    body[3] = 0x00;
    body[4] = 0x00;

    //写入NALU信息，右移8位，一个字节的读取？
    body[5] = (len >> 24) & 0xff;
    body[6] = (len >> 16) & 0xff;
    body[7] = (len >> 8) & 0xff;
    body[8] = (len) & 0xff;

    /*copy data*/
    memcpy(&body[9], buf, len);

    packet->m_hasAbsTimestamp = 0;
    packet->m_nBodySize = body_size;
    packet->m_packetType = RTMP_PACKET_TYPE_VIDEO;//当前packet的类型：Video
    packet->m_nChannel = 0x04;
    packet->m_headerType = RTMP_PACKET_SIZE_LARGE;
//	packet->m_nTimeStamp = -1;
    packet->m_nTimeStamp = RTMP_GetTime() - start_time;//记录了每一个tag相对于第一个tag（File Header）的相对时间
    add_rtmp_packet(packet);

}

extern "C"
JNIEXPORT void JNICALL
Java_com_example_xhp_mylive_jni_LiveNative_setAudioOptions(JNIEnv *env, jobject instance,
                                                           jint sampleRateInHz, jint channel) {
    //int errorStatus;
    CHANNEL_MODE channel_mode = MODE_2;
    if (aacEncOpen(&handle_aacencoder, 0, 2) != AACENC_OK) {
        //打开编码器失败
    }
    if (aacEncoder_SetParam(handle_aacencoder, AACENC_AOT, 2) != AACENC_OK) {
        //设置音频编码参数为低复杂度
    }
    if (aacEncoder_SetParam(handle_aacencoder, AACENC_SAMPLERATE, sampleRateInHz) != AACENC_OK) {
        //设置音频编码参数的频率
    }
    if (aacEncoder_SetParam(handle_aacencoder, AACENC_CHANNELMODE, channel_mode) != AACENC_OK) {
        //设置音频编码参数的通道个数：立体声
    }
    if (aacEncoder_SetParam(handle_aacencoder, AACENC_TRANSMUX, 2) != AACENC_OK) {
        LOGE("Unable to set the Raw transmux");
    }
//    if (aacEncoder_SetParam(handle_aacencoder, AACENC_CHANNELORDER, 1) != AACENC_OK) {
//        LOGE("Unable to set the bitrate");
//    }
//    if (aacEncoder_SetParam(handle_aacencoder, AACENC_BANDWIDTH, 0) != AACENC_OK) {
//        LOGE("Unable to set the AACENC_BANDWIDTH");//频宽
//    }
    int bitrate = 64 * 1000;
    if (aacEncoder_SetParam(handle_aacencoder, AACENC_BITRATE, bitrate) != AACENC_OK) {
        LOGE("Unable to set the bitrate");
    }
    if ((aacEncEncode(handle_aacencoder, NULL, NULL, NULL, NULL)) != AACENC_OK) {
        //打开音频编码器失败
        LOGE("打开音频编码器失败");
    } else {
        LOGD("打开音频编码器成功");
    }
    LOGD("设置音频参数完成");
    aacEncInfo(handle_aacencoder, &pInfo);


}

void getAACDecoderSpecificInfo(char spec_info[0], bool is_metadata) {
    //AACDecoderSpecificInfo
    //AAC | sample rate 44kHz | sample size 16bit | channels stereo
    //..1010............11..................1.................1
    //0x..A..............................F
    spec_info[0] = 0xAF;
    spec_info[1] = is_metadata ? 0x00 : 0x01;
}

/**
 * 添加AAC头信息
 */
void add_aac_sequence_header() {
    //获取aac头信息的长度
    //unsigned char *buf= (unsigned char *) malloc(2);
    unsigned long len = 2; //长度
    //faacEncGetDecoderSpecificInfo(audio_encode_handle,&buf,&len);
    int body_size = 2 + len;
    RTMPPacket *packet = (RTMPPacket *) malloc(sizeof(RTMPPacket));
    //RTMPPacket初始化
    RTMPPacket_Alloc(packet, body_size);
    RTMPPacket_Reset(packet);
    unsigned char *body = (unsigned char *) packet->m_body;
    //头信息配置
    /*AF 00 + AAC RAW data*/
    body[0] = 0xAF;//10 5 SoundFormat(4bits):10=AAC,SoundRate(2bits):3=44kHz,SoundSize(1bit):1=16-bit samples,SoundType(1bit):1=Stereo sound
    body[1] = 0x00;//AACPacketType:0表示AAC sequence header
    uint16_t audio_specific_config = 0;
    audio_specific_config |= ((2 << 11) & 0xF800);//2:AAC LC(Low Complexity)
    audio_specific_config |= ((4 << 7) & 0x0780);//4:44KHz
    audio_specific_config |= ((2 << 3) & 0x78);//2:Stereo
    //audio_specific_config |= 0 & 0x07;//padding:000
    body[2] = (audio_specific_config >> 8) & 0xFF;
    //body[3] = audio_specific_config & 0xFF;
    //memcpy(&body[2], buf, len); /*spec_buf是AAC sequence header数据*/
    packet->m_packetType = RTMP_PACKET_TYPE_AUDIO;
    packet->m_nBodySize = body_size;
    packet->m_nChannel = 0x04;
    packet->m_hasAbsTimestamp = 0;
    packet->m_nTimeStamp = 0;
    packet->m_headerType = RTMP_PACKET_SIZE_MEDIUM;
    add_rtmp_packet(packet);
    //free(buf);

}

/**
 * 添加AAC rtmp packet
 */
void add_aac_body(unsigned char *buf, int len) {
    int body_size = 2 + len;
    RTMPPacket *packet = (RTMPPacket *) malloc(sizeof(RTMPPacket));
    //RTMPPacket初始化
    RTMPPacket_Alloc(packet, body_size);
    RTMPPacket_Reset(packet);
    unsigned char *body = (unsigned char *) packet->m_body;
    //头信息配置
    /*AF 00 + AAC RAW data*/
    body[0] = 0xAF;//10 5 SoundFormat(4bits):10=AAC,SoundRate(2bits):3=44kHz,SoundSize(1bit):1=16-bit samples,SoundType(1bit):1=Stereo sound
    body[1] = 0x01;//AACPacketType:1表示AAC raw
    memcpy(&body[2], buf, len); /*spec_buf是AAC raw数据*/
    packet->m_packetType = RTMP_PACKET_TYPE_AUDIO;
    packet->m_nBodySize = body_size;
    packet->m_nChannel = 0x04;
    packet->m_hasAbsTimestamp = 0;
    packet->m_headerType = RTMP_PACKET_SIZE_LARGE;
    packet->m_nTimeStamp = RTMP_GetTime() - start_time;
    add_rtmp_packet(packet);
}

extern "C"
JNIEXPORT void JNICALL
Java_com_example_xhp_mylive_jni_LiveNative_sendAudio(JNIEnv *env, jobject instance,
                                                     jbyteArray data_, jint len) {
    jbyte *data = env->GetByteArrayElements(data_, NULL);
    AACENC_BufDesc in_buf = {0};
    {
        INT bid = IN_AUDIO_DATA;
        INT elSize = 2;
        void *inbuf = data;
        in_buf.bufs = &inbuf;
        in_buf.numBufs = 1;
        in_buf.bufferIdentifiers = &bid;
        in_buf.bufElSizes = &elSize;
    }

    uint8_t m_aacOutbuf[20480];
    AACENC_BufDesc out_buf = {0};
    {
        INT bid = OUT_BITSTREAM_DATA;
        void *buf = m_aacOutbuf;
        INT size = sizeof(m_aacOutbuf);
        INT elSize = 1;

        out_buf.bufs = &buf;
        out_buf.bufSizes = &size;
        out_buf.numBufs = 1;
        out_buf.bufferIdentifiers = &bid;
        out_buf.bufElSizes = &elSize;
    }

    AACENC_InArgs in_args = {0};
    in_args.numInSamples = (INT)len/2;

    AACENC_OutArgs out_args = {0};

    //AACENC_ERROR err;
    if ((aacEncEncode(handle_aacencoder, &in_buf, &out_buf, &in_args, &out_args)) !=
        AACENC_OK) {
        //break;
        //编码失败
        LOGE("音频编码失败");
    }
    if (out_args.numOutBytes == 0) {
        LOGE("无编码数据");
        //continue;
    } else {
        add_aac_body(m_aacOutbuf, out_args.numOutBytes);
    }

    env->ReleaseByteArrayElements(data_, data, 0);
}

/*
int channels = 2;
    int input_size = channels * 2 * pInfo.frameLength;
    uint8_t *input_buf = (uint8_t *) malloc(input_size);
    uint16_t *convert_buf = (uint16_t *) malloc(input_size);

        uint8_t m_aacOutbuf[20480];

    AACENC_BufDesc in_buf = {0}, out_buf = {0};
    AACENC_InArgs in_args = {0};
    AACENC_OutArgs out_args = {0};
    int in_identifier = IN_AUDIO_DATA;
    int in_elem_size = 2;
    void *inbuf = data;
    in_args.numInSamples = len / 2;  //size为pcm字节数
    in_buf.numBufs = 1;
    in_buf.bufs = &inbuf;  //pData为pcm数据指针
    in_buf.bufferIdentifiers = &in_identifier;
    in_buf.bufSizes = &len;
    in_buf.bufElSizes = &in_elem_size;

    int out_identifier = OUT_BITSTREAM_DATA;
    void *out_ptr = m_aacOutbuf;
    int out_size = sizeof(m_aacOutbuf);
    int out_elem_size = 1;
    out_buf.numBufs = 1;
    out_buf.bufs = &out_ptr;
    out_buf.bufferIdentifiers = &out_identifier;
    out_buf.bufSizes = &out_size;
    out_buf.bufElSizes = &out_elem_size;

        //AACENC_ERROR err;
        if ((aacEncEncode(handle_aacencoder, &in_buf, &out_buf, &in_args, &out_args)) !=
            AACENC_OK) {
            break;
            //编码失败
            LOGE("音频编码失败");
        }
        if (out_args.numOutBytes == 0) {
            //LOGE("无编码数据");
            continue;
        }
        add_aac_body(outbuf, out_args.numOutBytes);
 */