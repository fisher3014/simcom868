


#ifndef __BASE64_H_
#define __BASE64_H_

int Base64decode(char *bufplain, const char *bufcoded);
int Base64encode(char *encoded, const unsigned char *string, int len);

#endif  // __RS485_H_
