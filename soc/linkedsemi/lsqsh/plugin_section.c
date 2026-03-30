#include <plugin_section.h>

static volatile bool plugin_section_ready_flag;

extern char __plugin_start[];
extern char __plugin_end[];

bool in_plugin_section(uintptr_t entry)
{
    if (((uintptr_t)entry >= (uintptr_t)__plugin_start) && ((uintptr_t)entry < (uintptr_t)__plugin_end)) {
        return true;
    } else {
        return false;
    }
}

bool plugin_section_ready()
{
    return plugin_section_ready_flag;
}

void set_plugin_section_ready()
{
    plugin_section_ready_flag = true;
}

void set_plugin_section_unready()
{
    plugin_section_ready_flag = false;
}
