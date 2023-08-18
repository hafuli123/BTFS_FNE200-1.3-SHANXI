#include "algo_string.h"

#include "string.h"
#include "stdio.h"

//ÄÚ´æ²éÕÒ×Ö·û´®
char* memstr(char* full_data,int full_data_len, char* substr)
{
  int sublen;
  int i;
  char* cur;
  int last_possible;
  if (full_data == NULL || full_data_len <= 0 || substr == NULL) {
		return NULL;
  }
  if (*substr == '\0') {
		return NULL;
  }
	sublen = strlen(substr);
	last_possible = full_data_len - sublen + 1;
	cur = full_data;
  for (i = 0; i < last_possible; i++) {
		if (*cur == *substr) {
			if (memcmp(cur, substr, sublen) == 0) {
				return cur;
			}
		}
		cur++;
  }
  return NULL;
}
