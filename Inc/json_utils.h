#ifndef __JSON_UTILS_H
#define __JSON_UTILS_H

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#define STRING char *
#define uint unsigned int

#define VALUE_TYPE_NORMALPAIR 0

#define MEM_SIZE_STRUCTURE 720

typedef struct _JsonPair{

		char *key;
		char *value;
		int keyLength;
		int valueLength;	
		struct _JsonPair *next;
} JsonPair;

typedef struct _JsonObject{
		struct _JsonPair *jsonPair;
		struct _JsonPair *lastJsonPair;
	  	int length;
} JsonObject;

extern void OfoJsonCmbData( int sign, char* cmbdata, int *size, const char* key, char* value);
extern char *copyString(char *dec, char *src, int length);
//JsonObject *createJsonObject(void)£»
//char *structureMemFree();
#endif
