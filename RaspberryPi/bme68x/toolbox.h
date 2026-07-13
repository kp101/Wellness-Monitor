#if !defined(TOOLBOX_H)
#define TOOLBOX_H

    struct name_value
    {
       char *name;
       char value[20];
    };

    // check to see if string is of numeric value.
    extern bool is_numeric( char *p ); 

    // remove blanks from begining and end of string.
    extern char *crunch_blanks( char *p );

    // non-recursive, single level get value pair from a string of text in json format.
    extern int get_valuepair( const char * str , struct name_value *pair);


#endif

