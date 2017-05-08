#include "interpreter.h"
#include "vars.h"

#include <string.h>
#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

enum permission
{
	P_R 	= 0x01, // read
	P_W 	= 0x02, // write
	P_RW 	= 0x03, // equals P_R & P_W
};

enum token_id
{
	T_ERROR,
	
	// commands
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
	
	NUM_OF_VARS,
};

struct token
{
	enum token_id id;
	unsigned int value;
};

struct var_entry
{
	int *addr;
	char *symbol;
	enum permission perm;
	int min;
	int max;
};

//function prototypes
static bool is_number(char *string);
static enum var_type is_variable(char *string);
static bool eat(enum token_id tkn);
static struct token get_token(char *str);
static struct token next_token(void);

// them variables
static struct token current_tkn;

static struct var_entry variables[NUM_OF_VARS] =
{
	[V_NONE]   = { 0 },
	[V_TEMP]   = { &temperature, 		"TEMP",	  P_R,  0, 		0 },
	[V_TIMER]  = { &timer, 		 		"TIMER",  P_RW, 0, 		3600	},
	[V_STATUS] = { (int *)(&activated),	"STATUS", P_RW, false, 	true },
	[V_TARGET] = { &target,		 		"TARGET", P_RW, 40,	 	80 },
};

#define WRITABLE(V) 	(variables[V].perm & P_W)
#define READABLE(V) 	(variables[V].perm & P_R)
#define INBOUNDS(V, N) 	(((N) >= variables[V].min) && ((N) <= variables[V].max))

static bool is_number(char *string)
{
	char *strpos = string;
	
	if (*strpos == '\0') //empty string
		return false;
	
	while(*strpos)
		if (!isdigit(*strpos++))
			return false;
			
	return true;
}

static enum var_type is_variable(char *string)
{
        int i = 0;
	//skip V_NONE
	for (i = 1; i < sizeof(variables)/sizeof(*variables); i++)
		if (!strcmp(string, variables[i].symbol))
			return i;
			
	return 0; //no matching variable
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

static struct token get_token(char *str)
{
	enum var_type var;
	
	// really ugly if-else chain
	
	if (str == NULL)
		return (struct token) { T_END, 0 };
		
	else if (is_number(str))
		return (struct token) { T_INTEGER, strtol(str, NULL, 10) };
		
	else if ((var = is_variable(str)) != 0)
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

bool interpret(char *string, char *result)
{
	struct token var_tkn;
	
	char *strpos = string;
	if (*strpos) do
		*strpos = toupper(*strpos);
	while(*(++strpos));
		
	current_tkn = get_token(strtok(string, " "));
	
	if (eat(T_GET))
	{
		var_tkn = current_tkn;
		if (eat(T_VARIABLE))
		{
			sprintf(result, "%d", *(variables[var_tkn.value].addr));
			return true;
		}
                else
                  sprintf(result, "%s", "VAR");
	}
	else if (eat(T_SET))
	{
		var_tkn = current_tkn;
		if (eat(T_VARIABLE) && WRITABLE(var_tkn.value))
		{
			enum var_type value = current_tkn.value;
			if (eat(T_INTEGER) && INBOUNDS(var_tkn.value, value))
			{
				*(variables[var_tkn.value].addr) = value;
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
