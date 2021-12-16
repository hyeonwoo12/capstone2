#include "sha224.h"

//*********************************************************************************************************************************
// o SHA224_K	: �ܰ迬�꿡�� ���Ǵ� 64���� ���
//*********************************************************************************************************************************
const DWORD SHA224_K[64] = {0x428a2f98, 0x71374491, 0xb5c0fbcf, 0xe9b5dba5, 0x3956c25b, 0x59f111f1,
							0x923f82a4, 0xab1c5ed5, 0xd807aa98, 0x12835b01, 0x243185be, 0x550c7dc3,
							0x72be5d74, 0x80deb1fe, 0x9bdc06a7, 0xc19bf174, 0xe49b69c1, 0xefbe4786,
							0x0fc19dc6, 0x240ca1cc, 0x2de92c6f, 0x4a7484aa, 0x5cb0a9dc, 0x76f988da,
							0x983e5152, 0xa831c66d, 0xb00327c8, 0xbf597fc7, 0xc6e00bf3, 0xd5a79147,
							0x06ca6351, 0x14292967, 0x27b70a85, 0x2e1b2138, 0x4d2c6dfc, 0x53380d13,
							0x650a7354, 0x766a0abb, 0x81c2c92e, 0x92722c85, 0xa2bfe8a1, 0xa81a664b,
							0xc24b8b70, 0xc76c51a3, 0xd192e819, 0xd6990624, 0xf40e3585, 0x106aa070,
							0x19a4c116, 0x1e376c08, 0x2748774c, 0x34b0bcb5, 0x391c0cb3, 0x4ed8aa4a,
							0x5b9cca4f, 0x682e6ff3, 0x748f82ee, 0x78a5636f, 0x84c87814, 0x8cc70208,
							0x90befffa, 0xa4506ceb, 0xbef9a3f7, 0xc67178f2};

//*********************************************************************************************************************************
// o �����̼� �Լ�
//*********************************************************************************************************************************
#define RR(x, n)		ROTR_DWORD(x, n)

//*********************************************************************************************************************************
// o ����Ʈ �Լ�
//*********************************************************************************************************************************
#define SS(x, n)		(x >> n)

//*********************************************************************************************************************************
// o �ܰ迬�꿡 ���Ǵ� �ο��Լ�
//*********************************************************************************************************************************
#define Ch(x, y, z)		((x & y) ^ ((~x) & z))
#define Maj(x, y, z)	((x & y) ^ (x & z) ^ (y & z))
#define Sigma0(x)		(RR(x,  2) ^ RR(x, 13) ^ RR(x, 22))
#define Sigma1(x)		(RR(x,  6) ^ RR(x, 11) ^ RR(x, 25))

//*********************************************************************************************************************************
// o �޽��� Ȯ�忡 ���Ǵ� �Լ�
//*********************************************************************************************************************************
#define RHO0(x)			(RR(x,  7) ^ RR(x, 18) ^ SS(x,  3))
#define RHO1(x)			(RR(x, 17) ^ RR(x, 19) ^ SS(x, 10))

//*********************************************************************************************************************************
// o FF	: SHA-224�� �ܰ迬��
//*********************************************************************************************************************************
#define FF(a, b, c, d, e, f, g, h, j) {							\
		T1 = h + Sigma1(e) + Ch(e, f, g) + SHA224_K[j] + X[j];	\
		d += T1;												\
		h = T1 + Sigma0(a) + Maj(a, b, c);						\
}
		
//*********************************************************************************************************************************
// o SHA224_Transform()	: 512 ��Ʈ ���� ����� �޽����� �Է� �޾� ���⺯���� �����ϴ� ���� �Լ��ν�
//						  4 ����(64 �ܰ�)�� �����Ǹ� 8���� ���⺯��(a, b, c, d, e, f, g, h)�� ���
// o �Է�				: Message		- �Է� �޽����� ������ ����
//						  ChainVar		- ���⺯���� ������ ����
// o ���				: 
//*********************************************************************************************************************************
void SHA224_Transform(DWORD *Message, DWORD *ChainVar)
{
	DWORD a, b, c, d, e, f, g, h, T1, X[64];
	DWORD j;

#if defined(BIG_ENDIAN)
	#define GetData(x)	x
#else
	#define GetData(x)	ENDIAN_REVERSE_DWORD(x)
#endif

	// �޽��� Ȯ��
	for (j = 0; j < 16; j++)
		X[j] = GetData(Message[j]);

	for (j = 16; j < 64; j++)
		X[j] = RHO1(X[j - 2]) + X[j - 7] + RHO0(X[j - 15]) + X[j - 16];

	// ���⺯�� ����
	a = ChainVar[0];
	b = ChainVar[1];
	c = ChainVar[2];
	d = ChainVar[3];
	e = ChainVar[4];
	f = ChainVar[5];
	g = ChainVar[6];
	h = ChainVar[7];

	// �ܰ迬��
	for (j = 0; j < 64; j += 8)
	{
		FF(a, b, c, d, e, f, g, h, j + 0);
		FF(h, a, b, c, d, e, f, g, j + 1);
		FF(g, h, a, b, c, d, e, f, j + 2);
		FF(f, g, h, a, b, c, d, e, j + 3);
		FF(e, f, g, h, a, b, c, d, j + 4);
		FF(d, e, f, g, h, a, b, c, j + 5);
		FF(c, d, e, f, g, h, a, b, j + 6);
		FF(b, c, d, e, f, g, h, a, j + 7);
	}

	// ���⺯�� ����
	ChainVar[0] += a;
	ChainVar[1] += b;
	ChainVar[2] += c;
	ChainVar[3] += d;
	ChainVar[4] += e;
	ChainVar[5] += f;
	ChainVar[6] += g;
	ChainVar[7] += h;
}

//*********************************************************************************************************************************
// o SHA224_Init()		: ���⺯���� ���̺����� �ʱ�ȭ�ϴ� �Լ�
// o �Է�				: AlgInfo		-  SHA-224 ����ü�� ������ ����
// o ���				: 
//*********************************************************************************************************************************
void SHA224_Init(SHA224_ALG_INFO *AlgInfo)
{
	// ���⺯�� �ʱ�ȭ
	AlgInfo->ChainVar[0] = 0xc1059ed8;
	AlgInfo->ChainVar[1] = 0x367cd507;
	AlgInfo->ChainVar[2] = 0x3070dd17;
	AlgInfo->ChainVar[3] = 0xf70e5939;
	AlgInfo->ChainVar[4] = 0xffc00b31;
	AlgInfo->ChainVar[5] = 0x68581511;
	AlgInfo->ChainVar[6] = 0x64f98fa7;
	AlgInfo->ChainVar[7] = 0xbefa4fa4;

	// ���̺��� �ʱ�ȭ
	AlgInfo->Count[0] = AlgInfo->Count[1] = 0;
}

//*********************************************************************************************************************************
// o SHA224_Update()	: ������ ���̸� ������ �Է� �޽����� 512 ��Ʈ ��� ������ ������ �����Լ��� ȣ���ϴ� �Լ�
// o �Է�				: AlgInfo		- SHA-224 ����ü�� ������ ����
//						  Message		- �Է� �޽����� ������ ����
//						  MessageLen	- �Է� �޽����� ����Ʈ ����
// o ���				: 
//*********************************************************************************************************************************
void SHA224_Update(SHA224_ALG_INFO *AlgInfo, BYTE *Message, DWORD MessageLen)
{
	DWORD RemainedLen, PartLen;

	RemainedLen = (AlgInfo->Count[0] >> 3) % SHA224_DIGEST_BLOCKLEN;

	PartLen = SHA224_DIGEST_BLOCKLEN - RemainedLen;

	// �޽��� ���� ���� 32��Ʈ ���
	if ((AlgInfo->Count[0] += (MessageLen << 3)) < 0)
		AlgInfo->Count[1]++;

	// �޽��� ���� ���� 32��Ʈ ���
	AlgInfo->Count[1] += (MessageLen >> 29);

	// �Է� �޽����� 512 ��Ʈ ��� ������ ������ �����Լ��� ȣ��
	//	�ٽ� updtae �κ�
	if( MessageLen>=PartLen ) {
		memcpy(AlgInfo->Buffer+RemainedLen, Message, (int)PartLen);
		SHA224_Transform((DWORD *)AlgInfo->Buffer, AlgInfo->ChainVar);

		Message += PartLen;
		MessageLen -= PartLen;
		RemainedLen = 0;

		while( MessageLen>=SHA224_DIGEST_BLOCKLEN ) {
			if( (((int)Message)%4)==0 ) {	//	Speed up technique
				SHA224_Transform((DWORD *)Message, AlgInfo->ChainVar);
			}
			else {
				memcpy((BYTE *)AlgInfo->Buffer, Message, (int)SHA224_DIGEST_BLOCKLEN);
				SHA224_Transform((DWORD *)AlgInfo->Buffer, AlgInfo->ChainVar);
			}
			Message += SHA224_DIGEST_BLOCKLEN;
			MessageLen -= SHA224_DIGEST_BLOCKLEN;
		}
	}

	//	Buffer remaining input
	memcpy((BYTE *)AlgInfo->Buffer+RemainedLen, Message, (int)MessageLen);
}

//*********************************************************************************************************************************
// o SHA224_Final()		: �޽��� �����̱�� ���� �����̱⸦ ������ �� ������ �޽��� ����� ������ �����Լ��� ȣ���ϴ� �Լ�
// o �Է�				: AlgInfo	- SHA-224 ����ü�� ������ ����
//						  Digest	- SHA-224 �ؽ����� ������ ������ ����
// o ���				:
//*********************************************************************************************************************************
void SHA224_Final(SHA224_ALG_INFO *AlgInfo, BYTE *Digest)
{
	DWORD i, Index, CountL, CountH;

	// �޽��� ���� ���� ����
	CountL = AlgInfo->Count[0];
	CountH = AlgInfo->Count[1];

	// �޽��� �����̱�
	Index = (CountL >> 3) % SHA224_DIGEST_BLOCKLEN;
	AlgInfo->Buffer[Index++] = 0x80;

	// �����̱�� ���� �޽��� ����� 512 ��Ʈ�� ������ ���� �Լ��� ȣ��
	if (Index > SHA224_DIGEST_BLOCKLEN - 8)
	{
		memset((BYTE *)AlgInfo->Buffer + Index, 0, (int)(SHA224_DIGEST_BLOCKLEN - Index));
		SHA224_Transform((DWORD *)AlgInfo->Buffer, AlgInfo->ChainVar);
		memset((BYTE *)AlgInfo->Buffer, 0, (int)SHA224_DIGEST_BLOCKLEN - 8);
	}
	else
		memset((BYTE *)AlgInfo->Buffer + Index, 0, (int)(SHA224_DIGEST_BLOCKLEN - Index - 8));

	// �޽��� ���� ������ ����� ��ȯ
#if defined(LITTLE_ENDIAN)
	CountL = ENDIAN_REVERSE_DWORD(CountL);
	CountH = ENDIAN_REVERSE_DWORD(CountH);
#endif

	// �޽��� ���� �����̱�
	((DWORD *)AlgInfo->Buffer)[SHA224_DIGEST_BLOCKLEN / 4 - 2] = CountH;
	((DWORD *)AlgInfo->Buffer)[SHA224_DIGEST_BLOCKLEN / 4 - 1] = CountL;


	// ������ �޽��� ������� ���� �Լ��� ȣ��
	SHA224_Transform((DWORD *)AlgInfo->Buffer, AlgInfo->ChainVar);
	
	// ���� �ؽ����� Digest ������ ����
	for (i = 0; i < SHA224_DIGEST_VALUELEN; i += 4)
		BIG_D2B((AlgInfo->ChainVar)[i / 4], &(Digest[i]));
	

}