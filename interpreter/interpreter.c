#include "interpreter.h"
#include "vars.h"

#include <string.h>
#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

#define NONE 0

enum permission
{
	P_R 	= 0x01, // read
	P_W 	= 0x02, // write
	P_RW 	= 0x03, // equals P_R & P_W
};

enum token_id
{
	T_ERROR,
	
	T_GET,
	T_SET,
	
	T_VARIABLE,
	
	T_INTEGER,
	
	T_END,
	
	NUM_OF_TOKENS,
};

enum var_type
{
	V_NONE = 0,
	
	V_TEMP,
	V_TIMER,
	V_STATUS,
	V_TARGET,
};

struct token
{
	enum token_id id;
	unsigned int value;
};

struct variable
{
	int *addr;
	char *symbol;
	enum permission perm;
	int min;
	int max;
};

//function prototypes
static bool is_number(const char *string);
static enum var_type variable_type(const char *string);
static bool eat(enum token_id tkn);
static struct token get_token(const char *str);
static struct token next_token(void);
static void string_toupper(const char *s);

static struct token current_tkn;

static struct variable variables[] =
{
	[V_NONE]   = { 0 },
	[V_TEMP]   = { &temperature,		"TEMP",	  P_R,  0, 0 		},
	[V_TIMER]  = { &timer,				"TIMER",  P_RW, 0, 3600		},
	[V_STATUS] = { (int *)(&activated),	"STATUS", P_RW, false, true },
	[V_TARGET] = { &target,		 		"TARGET", P_RW, 40,	80 		},
};

#define WRITABLE(V) 	(variables[V].perm & P_W)
#define READABLE(V) 	(variables[V].perm & P_R)
#define INBOUNDS(V, N) 	(((N) >= variables[V].min) && ((N) <= variables[V].max))

static bool is_number(const char *string)
{
	char *strpos = string;
	
	if (*strpos == '\0') //empty string
		return false;
	
	while(*strpos)
		if (!isdigit(*strpos++))
			return false;
			
	return true;
}

static enum var_type variable_type(const char *string)
{
    int i = 0;

	//skip V_NONE
	for (i = 1; i < sizeof(variables)/sizeof(*variables); i++)
		if (!strcmp(string, variables[i].symbol))
			return i;
			
	return NONE; //no matching variable
}

static bool eat(enum token_id tkn)
{
	if (tkn == current_tkn.id)
	{
		current_tkn = next_token();
		return true;
	}
	else
		return false;
}

static struct token get_token(const char *str)
{
	enum var_type var;
	
	// really ugly if-else chain
	
	if (str == NULL)
		return (struct token) { T_END, 0 };
		
	else if (is_number(str))
		return (struct token) { T_INTEGER, strtol(str, NULL, 10) };
		
	else if ((var = variable_type(str)) != NONE)
		return (struct token) { T_VARIABLE, var };
		
	else if (!strcmp(str, "GET"))
		return (struct token) { T_GET, 0 };
		
	else if (!strcmp(str, "SET"))
		return (struct token) { T_SET, 0 };
		
	else
		return (struct token) { T_ERROR, 0 };
}

static struct token next_token(void)
{
	char *str = strtok(NULL, " ");
	
	return get_token(str);
}

static void string_toupper(const char *s)
{
	char *strpos = s;
	if (*strpos) do
		*strpos = toupper(*strpos);
	while(*(++strpos));
}

bool interpret(char *string, char *result)
{
	enum var_type var_val;
	enum var_type int_val;
	
	string_toupper(string);
		
	current_tkn = get_token(strtok(string, " "));
	
	if (eat(T_GET))
	{
		var_val = current_tkn.value;
		if (eat(T_VARIABLE))
		{
			sprintf(result, "%d", *(variables[var_val].addr));
			return true;
		}
        else
			sprintf(result, "%s", "VAR");
	}
	
	else if (eat(T_SET))
	{
		var_tkn = current_tkn;
		if (eat(T_VARIABLE) && WRITABLE(var_val))
		{
			int_val = current_tkn.value;
			if (eat(T_INTEGER) && INBOUNDS(var_val, int_val))
			{
				*(variables[var_val].addr) = int_val;
				sprintf(result, "%s", "OK");
				return true;
			}
            else
				sprintf(result, "%s", "INT");
		}
        else
			sprintf(result, "%s", "VAR");
	}
	else
		sprintf(result, "%s", "CMD");

	return false;
}
