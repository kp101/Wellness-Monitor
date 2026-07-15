#include <string.h>
#include <ctype.h>
#include <stdbool.h>
#include <stdio.h>

#include "toolbox.h"

bool is_numeric( char *p ) 
{

    while (*p != '\0') {
      if (isdigit(*p) || *p=='.')
         p++;
      else
         return false;
    }
    return true;
}

char *crunch_blanks( char *p ) 
{
    char *ptr = NULL;
    int i = 0;
    int len = strlen(p);

    for (i = len -1; i >=0 && p[i] == ' '; i--) 
        p[i] = '\0';
   
    len = strlen(p);
    for (i = 0; i < len && p[i] == ' '; i++) 
       ;
    if (i > 0) 
    {
       memmove(p, p+i,len-i);
       p[len-i]='\0';    
    }

    return p;
}

