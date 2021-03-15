// 图像特效实现

#include <math.h>
#include "pretty_effect.h"
#include "sdkconfig.h"
#include "decode_image.h"

// 解码图片到内存的地址指针
uint16_t **pixels;

// 从图片内存指针读取一个16位像素信息
static inline uint16_t get_bgnd_pixel(int x, int y)
{
	//Image has an 8x8 pixel margin, so we can also resolve e.g. [-3, 243]
	x+=8;
	y+=8;
	return pixels[y][x];
}

// 此变量用于检测下一帧标志位
static int prev_frame=-1;


//当画满一帧时，预先计算XY偏移系数，这样不用每个像素进行计算
static int8_t xofs[320], yofs[240];
static int8_t xcomp[320], ycomp[240];

// 计算一行特效，这里的一行不是一行像素，是16行像素作为一行一起发送显示
void CalcLineEffect(uint16_t *dest, int line, int frame, int linect)
{
	if (frame != prev_frame) {
		// 每当新一帧开始，就计算一组新的偏移系数，取一些随机的正弦作为偏移量，使图像看起来像水波纹的效果
		for (int x=0; x<320; x++) xofs[x]=sin(frame*0.15+x*0.06)*4;
		for (int y=0; y<240; y++) yofs[y]=sin(frame*0.1+y*0.05)*4;
		for (int x=0; x<320; x++) xcomp[x]=sin(frame*0.11+x*0.12)*4;
		for (int y=0; y<240; y++) ycomp[y]=sin(frame*0.07+y*0.15)*4;
		prev_frame = frame;
	}
	for (int y=line; y<line+linect; y++) {
		for (int x=0; x<320; x++) {// 将每个像素放在上面计算好的XY偏移坐标中
			*dest++=get_bgnd_pixel(x+yofs[y]+xcomp[x], y+xofs[x]+ycomp[y]);
		}
	}
}
// 特效初始化（把图片解码到内存中）
// 成功返回@ESP_OK，否则返回jpeg解码器的错误。
esp_err_t JPEG_Effect_init(void)
{
	return decode_image(&pixels);
}
