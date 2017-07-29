#include "json_utils.h"
static char *dst =NULL;

char *strdup(const char *src)
{
  int len;
  
  if (src == NULL)
    return NULL;
  
  if (src[0] == 0)
    return NULL;
  
  len = strlen(src) + 1;
	if(NULL != dst)
	{
		free(dst);
		dst = NULL;
	}
  dst = (char*)malloc(len);
  if (dst) 
    memcpy(dst, src, len);
  return dst;
}


char *copyString(char *dec, char *src, int length)
{				
	char *str = NULL;			
	memcpy(dec, src, length);		
	dec[length] = '\0';		
	str = dec;		
	return str;
}

void OfoJsonCmbData( int sign, char* cmbdata, int *size, const char* key, char* value)
{
	int index = *size;
	int key_len = strlen(key);
	int value_len = strlen(value);
	if(sign == 1) {
		cmbdata[index++] = '{';
		cmbdata[index++] = '"';
		memcpy(&cmbdata[index], strdup(key), key_len);
		index = index + key_len;
		cmbdata[index++] = '"';
		cmbdata[index++] = ':';
		cmbdata[index++] = '"';
		memcpy(&cmbdata[index], strdup(value), value_len);
		index = index + value_len;
		cmbdata[index++] = '"';
		cmbdata[index++] = ',';
	}
	else if(sign == 2){
		cmbdata[index++] = '"';
		memcpy(&cmbdata[index], strdup(key), key_len);
		index = index + key_len;
		cmbdata[index++] = '"';
		cmbdata[index++] = ':';
		cmbdata[index++] = '"';
		memcpy(&cmbdata[index], strdup(value), value_len);
		index = index + value_len;
		cmbdata[index++] = '"';
		cmbdata[index++] = '}';
		cmbdata[index++] = '\0';
		if(NULL != dst)
		{
			free(dst);
			dst = NULL;
		}

	} else {
		cmbdata[index++] = '"';
		memcpy(&cmbdata[index], strdup(key), key_len);
		index = index + key_len;
		cmbdata[index++] = '"';
		cmbdata[index++] = ':';
		cmbdata[index++] = '"';
		memcpy(&cmbdata[index], strdup(value), value_len);
		index = index + value_len;
		cmbdata[index++] = '"';
		cmbdata[index++] = ',';
	}
	*size = index;
}

