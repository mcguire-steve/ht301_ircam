// Xtherm.h : Xtherm DLL ����ͷ�ļ�
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
	//��������
	//width:��ȡ����ͼ������
	//height:ͼ������
	DLLPORT BOOL DataInit(int Width, int Height);


	//���²���
	//���ڿ���У�����������
	//type�����·�Χ 0��-20- 120  1�� 120 -400
	//pbuff:��ȡ����ͼ������
	DLLPORT void   UpdateParam(int type, BYTE* pbuff);

	//���»�����������
	//Emiss:Ŀ�귢����
	//refltmp:Ŀ�귴���¶�
	//airtmp:�����¶�
	//humi:����ʪ��
	//distance:Ŀ�����
	//fix:��������ֵ
	DLLPORT void   UpdateFixParam(float Emiss, float refltmp, float airtmp, float Humi, unsigned short Distance, float Fix);

	//��ȡ�豸������������
	//Emiss:Ŀ�귢����
	//refltmp:Ŀ�귴���¶�
	//airtmp:�����¶�
	//humi:����ʪ��
	//distance:Ŀ�����
	//fix:��������ֵ
	DLLPORT	void  GetFixParam(float* Emiss, float* refltmp, float* airtmp, float* Humi, unsigned short* Distance, float* Fix);


	//��ȡ�¶�����
	//type:�������� 0����ȫ֡�¶���� 1��ȫ֡�¶����
	//pbuff:��ȡ��������
	//maxtmp:����¶�ֵ
	//maxx,maxy:���ֵλ��
	//mintmp:��С�¶�ֵ
	//minx,miny:��Сֵλ��
	//centertmp:���ĵ��¶�ֵ
	//tmparr: ���������ֵ
	//alltmp:ȫ֡�¶����� ��type=1ʱ�����Ч
	DLLPORT void   GetTmpData(int type, BYTE* pbuff, float* maxtmp, int* maxx, int* maxy, float* mintmp, int* minx, int* miny, float* centertmp, float* tmparr, float* alltmp);

	//��ȡ��������
	 //fpatmp:̽�����¶�
	 //coretmp:��о�¶�
	 //fpaavg:̽���������ֵ
	 //orgavgԭʼ���������ֵ

	DLLPORT void   GetDevData(float* fpatmp, float* coretmp, int* fpaavg, int* orgavg);
}

#endif
