#include "interpreter.h"
#include "vars.h"

#include <string.h>
#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <limits.h>

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
	T_FIXED,
	T_STRING,
	
	T_END,
};

enum var_name
{
	V_NONE = 0,
	
	V_TEMP,
	V_TIMER,
	V_STATUS,
	V_TARGET,
	V_SSID,
	V_PASSWORD,
	V_IP,
};

union token_value
{
	int32_t num;
	char *string;
};

struct token
{
	enum token_id id;
	union token_value value;
};

struct variable
{
	void *addr;
	char *symbol;
	enum permission perm;
	enum token_id type;
	int min;
	int max;
};

//function prototypes
static bool is_integer(const char *string);
static enum var_name variable_type(const char *string);
static bool eat(enum token_id tkn);
static struct token get_token(const char *str);
static struct token next_token(void);
static void string_toupper(const char *s);

static void expect_value(enum var_name name, char *result);
static void set_variable(enum var_name name, union token_value val, char *result);
static void get_variable(enum var_name name, char *result);

static struct token current_tkn;
static char token_string[20];

static struct variable variables[] =
{
	[V_NONE]   = { 0 },
	[V_TEMP]   = { &temperature, "TEMP", P_R,  T_FIXED, 0, 0 },
	[V_TIMER]  = { &timer, "TIMER", P_RW, T_INTEGER, 0, INT_MAX },
	[V_STATUS] = { &activated,	"STATUS", P_RW, T_INTEGER, 0, 1 },
	[V_TARGET] = { &target, "TARGET", P_RW, T_INTEGER, 40, 80 },
	[V_SSID]   = { ssid, "SSID", P_RW, T_STRING, 0, 0, },
	[V_PASSWORD] = { password, "PASSWORD", P_W, T_STRING, 0, 0 },
	[V_IP] = { esp_ip, "IP", P_R, T_STRING, 0, 0 },
};

#define WRITABLE(V) 	(variables[V].perm & P_W)
#define READABLE(V) 	(variables[V].perm & P_R)
#define INBOUNDS(V, N) 	(((N) >= variables[V].min) && ((N) <= variables[V].max))

static bool is_integer(const char *string)
{
	const char *strpos = string;
	
	if (*strpos == '\0') //empty string
		return false;
	
	while(*strpos)
		if (!isdigit(*strpos++))
			return false;
			
	return true;
}

static bool is_fixed(const char *string)
{
	char *p = strchr(string, '.');
	if (p)
	{
		*p++ = '\0'; //replace character with end of string
		
		bool ret = is_integer(string) && is_integer(p) && strlen(p) == 1;
		*(--p) = '.';
		return ret;
	}
	else
		return false;
}

static int get_fixed(const char *string)
{
	char *p = strchr(string, '.');
	*p++ = '\0';
	int ret = 10 * strtol(string, NULL, 10) + strtol(p, NULL, 10);
	*(--p) = '.';
	
	return ret;
}

static enum var_name variable_type(const char *string)
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
	enum var_name var;
	
	// really ugly if-else chain
	
	if (str == NULL)
		return (struct token) { T_END, 0 };
		
	else if (is_integer(str))
		return (struct token) { T_INTEGER, strtol(str, NULL, 10) };
	
	else if (is_fixed(str))
		return (struct token) { T_FIXED, get_fixed(str) };
		
	else if ((var = variable_type(str)) != NONE)
		return (struct token) { T_VARIABLE, var };
		
	// a string begins and ends with quotation marks
	else if (str[0] == '"' && str[strlen(str) - 1] == '"')
	{
		//don't include first quotation mark
		strcpy(token_string, str + 1);
		
		//remove trailing quotation mark
		token_string[strlen(token_string) - 1] = '\0';
		return (struct token) { T_STRING, {.string = token_string} };
	}
		
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

static void expect_value(enum var_name name, char *result)
{
	int type = variables[name].type; //expected type
	eat(T_VARIABLE);
	
	if (current_tkn.id == type)
		set_variable(name, current_tkn.value, result);
	else
		sprintf(result, "%s", "NOT A VARIABLE");
}

static void set_variable(enum var_name name, union token_value val, char *result)
{
	if (!WRITABLE(name))
		sprintf(result, "%s", "NOT WRITABLE");
		
	else switch(variables[name].type)
	{
	case T_INTEGER:
	case T_FIXED:
		if (INBOUNDS(name, val.num))
		{
			*(int32_t *)(variables[name].addr) = val.num;
			sprintf(result, "%s", "OK");
		}
		else
			sprintf(result, "%s", "OOB");
		break;
	case T_STRING:
		strcpy(variables[name].addr, val.string);
		sprintf(result, "%s", "OK");
		break;
	default:
		sprintf(result, "%s", "UNVALID VALUE");
		return;
	}
	
}

static void get_variable(enum var_name name, char *result)
{
	int num;
	if (!READABLE(name))
		sprintf(result, "%s", "NOT READABLE");

	else switch(variables[name].type)
	{
	case T_INTEGER:
		sprintf(result, "%d", *(int32_t *)variables[name].addr);
		break;
	case T_STRING:
		sprintf(result, "%s", (char *)variables[name].addr);
		break;
	case T_FIXED:
		num = *(int *)variables[name].addr;
		sprintf(result, "%d.%d", num / 10, num % 10);
		break;
	default:
		sprintf(result, "%s", "UNVALID VALUE");
	}
}

bool interpret(char *string, char *result)
{
	struct token tkn;
		
	current_tkn = get_token(strtok(string, " "));
	
	if (eat(T_GET) && current_tkn.id == T_VARIABLE)
		get_variable(current_tkn.value.num, result);

	else if (eat(T_SET) && current_tkn.id == T_VARIABLE)
		expect_value(current_tkn.value.num, result);

	else
		sprintf(result, "%s", "NOT A VALID COMMAND");
		
	return true;
}
