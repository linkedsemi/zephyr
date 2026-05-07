#ifndef _PLUGIN_SECTION_H_
#define _PLUGIN_SECTION_H_

#include <stdint.h>
#include <stdbool.h>

bool in_plugin_section(uintptr_t entry);
bool plugin_section_ready();
void set_plugin_section_ready();
void set_plugin_section_unready();

#endif /* _PLUGIN_SECTION_H_ */
