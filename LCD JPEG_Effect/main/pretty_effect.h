
#pragma once
#include <stdint.h>
#include "esp_err.h"

/**
  * @brief 计算16行像素特效
  * @param dest 计算好的像素缓存地址 16 * 320 16位像素值。
  * @param line 此行在液晶起始行（y坐标）
  * @param frame 当前帧号，用于上下帧的动画计算
  * @param linect 计算的行数
  */
void CalcLineEffect(uint16_t *dest, int line, int frame, int linect);


// 特效初始化（把图片解码到内存中）
// 成功返回@ESP_OK，否则返回jpeg解码器的错误。
esp_err_t JPEG_Effect_init(void);
