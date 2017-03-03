tar rem 127.0.0.1:2331
file kernel.elf
define dump_calls
	set $start=__rnk_initcalls_start
	while ($start < __rnk_initcalls_end)
		x/1xw $start
		set $start=$start+4
	end
end
