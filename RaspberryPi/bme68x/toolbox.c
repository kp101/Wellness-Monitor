#include <string.h>
#include <ctype.h>
#include <stdbool.h>
#include <stdio.h>

#include "toolbox.h"

bool is_numeric( char *p ) {

   while (*p != '\0') {
     if (isdigit(*p) || *p=='.')
        p++;
     else
        return false;
   }
   return true;
}

char *crunch_blanks( char *p ) {
   char *ptr = NULL;
   int i = 0;
   int len = strlen(p);

   for (i = len -1; i >=0 && p[i] == ' '; i--) 
        p[i] = '\0';
   
   len = strlen(p);
   for (i = 0; i < len && p[i] == ' '; i++) 
      ;
   if (i > 0) {
      memmove(p, p+i,len-i);
      p[len-i]='\0';    
   }

   return p;
}

int get_valuepair( const char * str , struct name_value *pair)
{
    const char outer_del[] = "{,}";
    const char inner_del[] = ":";

    char *token;
    char *outer_ptr = NULL;
    char *inner_ptr = NULL;
    char buff[strlen(str)+1];
   
    pair->value[0] = '\0';
    strcpy(buff, str);
    token = strtok_r( buff, outer_del, &outer_ptr);

    while(token != NULL)
    {
        char *inner_token = strtok_r( token, inner_del, &inner_ptr);
        int name = 1;

        while(inner_token != NULL)
        {
            if (name && strstr(inner_token, pair->name) != NULL )
            {
                inner_token = strtok_r(NULL, inner_del, &inner_ptr);
                if (inner_token !=NULL)
                {
                    strcpy( pair->value, inner_token );
                }
            }
            else
                inner_token = strtok_r(NULL, inner_del, &inner_ptr);
            name = !name;
        }
        token = strtok_r(NULL, outer_del, &outer_ptr);
    }
    return 0;
}

