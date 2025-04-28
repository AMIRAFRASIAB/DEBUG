
#include "serial_hmi.h"
#include "serial_debugger.h"
#include "string.h"

//------------------------------------------------------------------------
typedef void (*vFn) (void);
typedef struct {
  const char* CMD;
  vFn         fn;
} dispatchItem_s;
//------------------------------------------------------------------------
#define __ADD_CMD(identifier, string)  __attribute__((used)) const char identifier[] = string;static void run_ ## identifier (void)
#define ADD_CMD(...)                   __ADD_CMD(__VA_ARGS__)
#define EXPAND_CMD(cmd)                (dispatchItem_s){.CMD = cmd,     .fn = run_ ## cmd }
//------------------------------------------------------------------------
#define MAX_POSSIBLE_SIMILAR_CMDS     4
static char* argv[20] = {[0 ... 19] = NULL};
static uint8_t argc   = 0;
//------------------------------------------------------------------------
/* Command list */
ADD_CMD(  CMD1       ,"CMD1"   );  
//------------------------------------------------------------------------
static const dispatchItem_s LIST[] = {
  EXPAND_CMD(   CMD1       ),
};
//------------------------------------------------------------------------
/* Private APIs */
static void __hmi_update_tokens (const char* ARGS, uint8_t strLen) {
  for (uint8_t i = 0; i < sizeof(argv) / sizeof(argv[0]); i++) {
    argv[i] = NULL;
  }
  argc = 0;
  char* args = (void*)ARGS;
  args[strLen - 1] = '\0';
  char* tok = strtok(args," ");
  while (tok != NULL) {
    argv[argc++] = tok;
    tok = strtok(NULL, " ");
    argc %= (sizeof(argv) / sizeof(*argv));
  }
}
//------------------------------------------------------------------------
/* Public APIs */
void hmi_decoder (void* stream, uint16_t streamLen) {
  const dispatchItem_s* matchedItems[MAX_POSSIBLE_SIMILAR_CMDS] = {NULL};
  uint8_t matchCounter = 0;
  char* input = (char*)stream;
  uint8_t len;
	input[streamLen - 1] = '\0';
  for (uint32_t i = 0; i < sizeof(LIST) / sizeof(*LIST); i++) {
		len = strlen(LIST[i].CMD);
    if (!strncmp(LIST[i].CMD, input, strlen(LIST[i].CMD))) {
      matchedItems[matchCounter++] = LIST + i;
      if (matchCounter >= sizeof(matchedItems) / sizeof(*matchedItems)) {
        LOG_ERROR("HMI :: Too many similar cmds");
        return;
      }
    }
  }
  const dispatchItem_s* targetItem = matchedItems[0];
  for (uint8_t i = 1; i < matchCounter; i++) {
    if (strlen(targetItem->CMD) < strlen(matchedItems[i]->CMD)) {
      targetItem = matchedItems[i];
    }
  }
  if (targetItem != NULL) {
    len = strlen(targetItem->CMD);
    __hmi_update_tokens(input + len, streamLen - len);
    targetItem->fn();
  }
  else {
    /* No Command Matched */
    LOG_WARNING("HMI :: cmd not found");
  }
  return;
}
//------------------------------------------------------------------------
/* Callback functions implementation */
void run_CMD1 (void) {}
