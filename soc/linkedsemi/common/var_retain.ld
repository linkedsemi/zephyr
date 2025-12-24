SECTION_PROLOGUE(.var_retain,(ORIGIN(RAM)+LENGTH(RAM)-var_retain_section_size) (NOLOAD),)
{
	*(SORT_BY_NAME(.var_retain*))
}
var_retain_section_size = SIZEOF(.var_retain);