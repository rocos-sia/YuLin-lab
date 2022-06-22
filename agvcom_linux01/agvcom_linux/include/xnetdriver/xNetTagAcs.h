 /*
 *  file : xnettagacs.h
 *  author : TimXu,AllenWang
 *  note : read/write tag from controller
 */

#ifndef _H_XNETTAGACS
#define _H_XNETTAGACS

#ifdef _WINDOWS
#ifdef _DLL_XNET_DRIVER
#define XNET_DRIVER_EXPORT __declspec(dllexport)
#else
#define XNET_DRIVER_EXPORT __declspec(dllimport)
#endif
#else
#define XNET_DRIVER_EXPORT
#endif

#ifdef __cplusplus
extern "C" {
#endif  /* __cplusplus */

	// set ip address
	void XNET_DRIVER_EXPORT xdriver_set_ip(unsigned char b1,unsigned char b2,unsigned char b3,unsigned char b4);

	// read/write in memory format
	bool XNET_DRIVER_EXPORT xdriver_get(const char *tagName,void *pData,unsigned int len ,int nTime=2000);
	bool XNET_DRIVER_EXPORT xdriver_set(const char *tagName,const void *pData,unsigned int len,int nTime=2000);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif
