#include <ht301_ircam/XthermDll.h>
#include <math.h>
#include <string.h>

float Fix_, Distance_, refltmp_, airtmp_, Humi_, Emiss_;
float fpatmp_, fpaavg_, orgavg_, coretmp_;
float flt_100133AC, flt_100133A8, flt_100033A0;
float flt_10003360, flt_1000335C, flt_1000339C, flt_100033A4, flt_10003398;
float flt_10003394, temperatureLUT[16384], flt_10003378, flt_1000337C;
int type_, dev_type_, Height_, Width_;
static int readParaFromDevFlag = 1;

void __cdecl sub_10001010()
{
	float v0; // ST08_4
	float v1; // ST04_4
	float v2; // ST08_4
	float v3; // ST08_4
	float v4; // ST0C_4
	float v5; // ST0C_4
	double v6; // ST10_8
	float v7; // ST0C_4
	float v8; // ST0C_4
	float v9; // ST0C_4
	double v10; // ST10_8
	float v11; // ST0C_4

	v0 = exp(
		airtmp_ * (airtmp_ * 0.00000068455 * airtmp_)
		+ 0.06938999999999999 * airtmp_
		+ 1.5587
		- airtmp_ * 0.00027816 * airtmp_)
		* Humi_;
	v1 = sqrt(v0);
	v2 = sqrt((double)(unsigned __int16)Distance_);
	v3 = -v2;
	v4 = (0.006568999961018562 - v1 * 0.00227600010111928) * v3;
	v5 = exp(v4);
	v6 = v5 * 1.899999976158142;
	v7 = (0.01262000016868114 - v1 * 0.006670000031590462) * v3;
	v8 = exp(v7);
	flt_100133AC = v6 - v8 * 0.8999999761581421;
	flt_100133A8 = 1.0 / (Emiss_ * flt_100133AC);
	v9 = pow(refltmp_ + 273.15, 4.0);
	v10 = v9 * (1.0 - Emiss_) * flt_100133AC;
	v11 = pow(airtmp_ + 273.15, 4.0);
	flt_100033A0 = v11 * (1.0 - flt_100133AC) + v10;
}

DLLPORT void   UpdateFixParam(float Emiss, float refltmp, float airtmp, float Humi, unsigned short Distance, float Fix)
{
	Fix_ = Fix;
	Distance_ = Distance;
	refltmp_ = refltmp;
	airtmp_ = airtmp;
	Humi_ = Humi;
	Emiss_ = Emiss;
	sub_10001010();
}

DLLPORT	void  GetFixParam(float* Emiss, float* refltmp, float* airtmp, float* Humi, unsigned short* Distance, float* Fix)
{
	*Fix = Fix_;
	*refltmp = refltmp_;
	*airtmp = airtmp_;
	*Humi = Humi_;
	*Emiss = Emiss_;
	*Distance = Distance_;
	sub_10001010();
}

unsigned int __cdecl sub_10001180(float a1, __int16 cx)
{
	__int16 v2; // ax
	uint16_t v3; // bx
	uint16_t v4; // cx
	int v5; // edi
	float* p; // esi
	float v7; // ST14_4
	float v8; // ST14_4
	float v9; // ST14_4
	unsigned int result; // eax
	float v11; // ST14_4
	double v12; // st7
	float v13; // ST14_4
	float v14; // ST14_4
	float v15; // ST14_4
	double v16; // st7
	float v17; // [esp+4h] [ebp-10h]
	float v18; // [esp+4h] [ebp-10h]
	int v19; // [esp+8h] [ebp-Ch]
	float v20; // [esp+8h] [ebp-Ch]
	signed int v21; // [esp+8h] [ebp-Ch]
	float v22; // [esp+Ch] [ebp-8h]
	float v23; // [esp+10h] [ebp-4h]

	v23 = flt_10003360 * a1 * a1 + a1 * flt_1000335C;
	v22 = flt_1000339C * flt_100033A4 * flt_100033A4 + flt_10003398 * flt_100033A4 + flt_10003394;
	if (type_)
		v2 = 0;
	else
		v2 = (signed int)(390.0 - flt_100033A4 * 7.05);
	v3 = Distance_;
	v4 = cx - v2;
	v5 = -v4;
	v19 = -v4;
	p = temperatureLUT;
	while (p - temperatureLUT < 16384)
	{
		v7 = (double)v19 * v22 + v23;
		v8 = v7 / flt_10003360 + flt_10003378;
		v9 = sqrt(v8);
		result = 4;
		v11 = v9 - flt_1000337C;
		v20 = v11 + 273.1499938964844;
		v17 = 1.0;
		while (1)
		{
			v12 = v20;
			if (result & 1)
				v17 = v17 * v12;
			result >>= 1;
			if (!result)
				break;
			v20 = v12 * v12;
		}
		v13 = v17 - flt_100033A0;
		v14 = v13 * flt_100133A8;		v15 = pow(v14, 0.25);
		v18 = v15 - 273.15;
		if (v3 >= 20)
			v21 = 20;
		else
			v21 = v3;
		++v5;
		v16 = (double)v21 * 0.85;
		v19 = v5;
		*p = v18 + (v16 - 1.125) * (v18 - airtmp_) / 100.0;
		++p;
	}
	return result;
}

DLLPORT void   UpdateParam(int type, BYTE* pbuff)
{
	int v2; // eax
	int v3; // eax
	double v4; // st7
	int v5; // esi
	float v6; // edi
	int v7; // edx
	float v8; // edi
	float v9; // edi
	float v10; // eax
	int v11; // edx
	float v12; // eax
	float v13; // eax
	int typeb; // [esp+10h] [ebp+4h]
	int typea; // [esp+10h] [ebp+4h]

	type_ = type;
	v2 = Height_ + 3;
	if (!dev_type_)
		v2 = Height_ + 1;
	v3 = Width_ * v2;
	v4 = (double)(*(unsigned __int16*)& pbuff[2 * Width_ * Height_ + 2] - 7800) / 36.0;
	v5 = *(unsigned __int16*)& pbuff[2 * v3];
	typeb = *(unsigned __int16*)& pbuff[2 * v3 + 2];
	v6 = *(float*)& pbuff[2 * v3 + 6];
	v7 = v3 + 127;
	v3 += 5;
	flt_10003360 = v6;
	v8 = *(float*)& pbuff[2 * v3];
	v3 += 2;
	flt_1000335C = v8;
	v9 = *(float*)& pbuff[2 * v3];
	v3 += 2;
	flt_1000339C = v9;
	flt_10003398 = *(float*)& pbuff[2 * v3];
	flt_10003394 = *(float*)& pbuff[2 * v3 + 4];
	flt_100033A4 = 20.0 - v4;
	*(float*)& typea = (double)typeb / 10.0 - 273.1499938964844;
	if (readParaFromDevFlag)
	{
		Fix_ = *(float*)& pbuff[2 * v7];
		v10 = *(float*)& pbuff[2 * v7 + 4];
		v11 = v7 + 2;
		refltmp_ = v10;
		v12 = *(float*)& pbuff[2 * v11 + 4];
		v11 += 2;
		airtmp_ = v12;
		v13 = *(float*)& pbuff[2 * v11 + 4];
		v11 += 4;
		Humi_ = v13;
		Emiss_ = *(float*)& pbuff[2 * v11];
		Distance_ = *(unsigned short*)& pbuff[2 * v11 + 4];
		readParaFromDevFlag = 0;
	}
	flt_1000337C = flt_1000335C / (flt_10003360 + flt_10003360);
	flt_10003378 = flt_1000335C * flt_1000335C / (flt_10003360 * (4.0 * flt_10003360));
	sub_10001010();
	sub_10001180(*(float*)& typea, v5);//bug in IDA
}

DLLPORT BOOL DataInit(int Width, int Height)
{
	BOOL result; // eax

	if (Width > 640)
	{
		if (Width == 1024)
		{
			dev_type_ = 3;
			Width_ = 1024;
			Height_ = 768;
		}
		else if (Width == 1280)
		{
			Width_ = 1280;
			dev_type_ = 4;
			Height_ = 1024;
			return 1;
		}
		result = 1;
	}
	else if (Width == 640)
	{
		dev_type_ = 2;
		Width_ = 640;
		Height_ = 512;
		result = 1;
	}
	else if (Width == 240)
	{
		dev_type_ = 0;
		Width_ = 240;
		Height_ = 180;
		result = 1;
	}
	else
	{
		result = 1;
		if (Width == 384)
		{
			dev_type_ = 1;
			Width_ = 384;
			Height_ = 288;
		}
	}
	return result;
}

DLLPORT void   GetTmpData(int type, BYTE* pbuff, float* maxtmp, int* maxx, int* maxy, float* mintmp, int* minx, int* miny, float* centertmp, float* tmparr, float* alltmp)
{
	unsigned char* v11; // ebx
	int v12; // edi
	uint16_t* v13; // eax
	signed int v14; // edx
	int v15; // ecx
	int v16; // edx
	float* v17; // ecx
	int v18; // esi
	float* v19; // ecx
	unsigned int v20; // edx
	uint64_t v21; // eax
	int v22; // ebp
	int v23; // edx
	unsigned char* pbuffa; // [esp+10h] [ebp+8h]

	v11 = pbuff;
	v12 = Width_ * Height_; //size of image
	v13 = (uint16_t*)& pbuff[2 * Width_ * Height_]; //start of metadata
	v14 = 3;
	fpatmp_ = 20.0 - ((double) * (uint16_t*)& pbuff[2 * Width_ * Height_ + 2] - 7800.0) / 36.0;
	if (!dev_type_)
		v14 = 1;
	pbuffa = (unsigned char*)v13[v14 * Width_ + 2]; //starts at the third line, plus 2 chars of metadata
	v15 = v13[8]; 
	fpaavg_ = *v13;
	v16 = v13[12];
	orgavg_ = v15;
	coretmp_ = (double)((uint64_t)pbuffa) / 10.0 - 273.1; 
	*centertmp = temperatureLUT[v16] + Fix_;
	*maxtmp = temperatureLUT[v13[4]] + Fix_;
	*mintmp = temperatureLUT[v13[7]] + Fix_;
	*maxx = v13[2];
	*maxy = v13[3];
	*minx = v13[5];
	*miny = v13[6];
	*tmparr = temperatureLUT[v13[13]] + Fix_;
	tmparr[1] = temperatureLUT[v13[14]] + Fix_;
	tmparr[2] = temperatureLUT[v13[15]] + Fix_;
	if (!type)
	{
	  memcpy(alltmp, temperatureLUT, sizeof(temperatureLUT));
	}
	    /*
		v17 = alltmp;
		v18 = 0;
		if (v12 >= 4)
		{
			v19 = alltmp + 2;
			v20 = ((unsigned int)(v12 - 4) >> 2) + 1;
			v21 = (uint64_t)(v11 + 4);
			v18 = 4 * v20;
			do
			{
				v22 = *(uint16_t*)(v21 - 4);
				v21 += 8;
				v19 += 4;
				--v20;
				*(v19 - 6) = temperatureLUT[v22] + Fix_;
				*(v19 - 5) = temperatureLUT[*(uint16_t*)(v21 - 10)] + Fix_;
				*(v19 - 4) = temperatureLUT[*(uint16_t*)(v21 - 8)] + Fix_;
				*(v19 - 3) = temperatureLUT[*(uint16_t*)(v21 - 6)] + Fix_;
			} while (v20);
			v17 = alltmp;
		}
		for (; v18 < v12; v17[v18 - 1] = temperatureLUT[v23] + Fix_)
			v23 = *(uint16_t*)& v11[2 * v18++];
	}
	    */
	    
}

DLLPORT void   GetDevData(float* fpatmp, float* coretmp, int* fpaavg, int* orgavg)
{
	*fpatmp = fpatmp_;
	*coretmp = coretmp_;
	*fpaavg = fpaavg_;
	*orgavg = orgavg_;
}
