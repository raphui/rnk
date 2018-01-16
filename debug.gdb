define dump_calls
	set $start=__rnk_initcalls_start
	while ($start < __rnk_initcalls_end)
		x/1xw $start
		set $start=$start+4
	end
end

define enable_swo
	monitor SWO EnableTarget 0 64000 1 0
end

define enable_semihosting
	monitor semihosting enable
end

define load_kernel
	load kernel.elf
	moni reset 0
	c
end

define load_apps
	if $argc == 0
		print "specify apps path"
	end

	if $argc == 1
		restore $arg0 binary 0x08050000
	end
end

tar rem 127.0.0.1:2331
file kernel.elf
