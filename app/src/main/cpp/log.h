/*
 * log.h
 *
 *  Created on: 2016年11月6日
 *      Author: XHP
 */
#include<android/log.h>
#ifndef LOG_TAG
#define LOG_TAG "NativeLog"
#define LOGD(...) __android_log_print(ANDROID_LOG_DEBUG,LOG_TAG,__VA_ARGS__)
#define LOGI(...) __android_log_print(ANDROID_LOG_INFO,LOG_TAG,__VA_ARGS__)
#define LOGW(...) __android_log_print(ANDROID_LOG_WARN,LOG_TAG,__VA_ARGS__)
#define LOGE(...) __android_log_print(ANDROID_LOG_ERROR,LOG_TAG,__VA_ARGS__)
#endif /* LOG_TAG */

