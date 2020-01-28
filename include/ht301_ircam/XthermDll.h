// Xtherm.h : Xtherm DLL 的主头文件
//
#ifndef XTHERMDLL_H_
#define XTHERMDLL_H_

#define DLLPORT //_declspec(dllimport)

#include <stdint.h>
#define __cdecl __attribute__((__cdecl__))
//#include <windows.h>
typedef bool BOOL;
typedef uint8_t BYTE;
typedef short __int16;

extern "C"  //C++
{
	//加载数据
	//width:获取到的图像列数
	//height:图像行数
	DLLPORT BOOL DataInit(int Width, int Height);


	//更新参数
	//用于快门校正后参数更新
	//type：测温范围 0：-20- 120  1： 120 -400
	//pbuff:获取到的图像数据
	DLLPORT void   UpdateParam(int type, BYTE* pbuff);

	//更新环境修正参数
	//Emiss:目标发射率
	//refltmp:目标反射温度
	//airtmp:环境温度
	//humi:环境湿度
	//distance:目标距离
	//fix:测温修正值
	DLLPORT void   UpdateFixParam(float Emiss, float refltmp, float airtmp, float Humi, unsigned short Distance, float Fix);

	//读取设备环境修正参数
	//Emiss:目标发射率
	//refltmp:目标反射温度
	//airtmp:环境温度
	//humi:环境湿度
	//distance:目标距离
	//fix:测温修正值
	DLLPORT	void  GetFixParam(float* Emiss, float* refltmp, float* airtmp, float* Humi, unsigned short* Distance, float* Fix);


	//获取温度数据
	//type:测温类型 0：无全帧温度输出 1：全帧温度输出
	//pbuff:获取到的数据
	//maxtmp:最大温度值
	//maxx,maxy:最大值位置
	//mintmp:最小温度值
	//minx,miny:最小值位置
	//centertmp:中心点温度值
	//tmparr: 三个点测温值
	//alltmp:全帧温度数据 仅type=1时输出有效
	DLLPORT void   GetTmpData(int type, BYTE* pbuff, float* maxtmp, int* maxx, int* maxy, float* mintmp, int* minx, int* miny, float* centertmp, float* tmparr, float* alltmp);

	//获取基本数据
	 //fpatmp:探测器温度
	 //coretmp:机芯温度
	 //fpaavg:探测器输出均值
	 //orgavg原始数据输出均值

	DLLPORT void   GetDevData(float* fpatmp, float* coretmp, int* fpaavg, int* orgavg);
}

#endif
