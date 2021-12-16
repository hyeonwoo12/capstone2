#ifndef _SHA256_H
#define _SHA256_H

#ifdef  __cplusplus
extern "C" {
#endif

#include "type.h"

	// SHA-256 ��� ����
#define SHA256_DIGEST_BLOCKLEN	64
#define SHA256_DIGEST_VALUELEN	32

// SHA-256 ����ü ����
	typedef struct {
		unsigned int ChainVar[SHA256_DIGEST_VALUELEN / 4];	// ���⺯��
		unsigned int Count[4];								// ���̺���
		unsigned char Buffer[SHA256_DIGEST_BLOCKLEN];		// �޽��� ����
	} SHA256_ALG_INFO;

	/*************** Functions ************************************************/

	// SHA-256 �Լ� ����
	void SHA256_Init(SHA256_ALG_INFO *AlgInfo);
	void SHA256_Update(SHA256_ALG_INFO *AlgInfo, unsigned char* Message, unsigned int MessageLen);
	void SHA256_Final(SHA256_ALG_INFO *AlgInfo, unsigned char* Digest);

	void SHA256_GeneralTest();

	/*************** END OF FILE **********************************************/
#ifdef  __cplusplus
}
#endif

#endif	//	_SHA256_H
